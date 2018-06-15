/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2011-2018 ARM or its affiliates
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2.
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*/

/*
 * ACamera PCI Express UIO driver
 *
 */
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/uio_driver.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/signal.h>
#include <linux/of_reserved_mem.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include "acamera_firmware_config.h"
#include "acamera_logger.h"
#include "system_hw_io.h"
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <asm/unaligned.h>
#include <linux/delay.h>


#define LOG_CONTEXT "[ ACamera ]"

#define ISP_V4L2_MODULE_NAME "isp-v4l2"

#include "v4l2_interface/isp-v4l2.h"

#define AO_RTI_GEN_PWR_SLEEP0 	(0xff800000 + 0x3a * 4)
#define AO_RTI_GEN_PWR_ISO0		(0xff800000 + 0x3b * 4)
#define HHI_ISP_MEM_PD_REG0		(0xff63c000 + 0x45 * 4)
#define HHI_ISP_MEM_PD_REG1		(0xff63c000 + 0x46 * 4)
#define HHI_CSI_PHY_CNTL0		(0xff630000 + 0xd3 * 4)
#define HHI_CSI_PHY_CNTL1		(0xff630000 + 0x114 * 4)

extern uint8_t *isp_kaddr;
extern resource_size_t isp_paddr;

extern void system_interrupts_set_irq( int irq_num, int flags );
extern void system_interrupts_init(void);

//map and unmap fpga memory
extern int32_t init_hw_io( resource_size_t addr, resource_size_t size );
extern int32_t close_hw_io( void );

static struct v4l2_device v4l2_dev;
static struct platform_device *isp_pdev;
static int initialized = 0;

#if V4L2_SOC_SUBDEV_ENABLE
extern uint32_t fw_calibration_update( void );
#include "soc_iq.h"

static int v4l2devs_running = 0;

struct acamera_v4l2_subdev_t {

    struct v4l2_subdev *soc_subdevs[V4L2_SOC_SUBDEV_NUMBER];
    struct v4l2_async_subdev soc_async_sd[V4L2_SOC_SUBDEV_NUMBER];
    struct v4l2_async_subdev *soc_async_sd_ptr[V4L2_SOC_SUBDEV_NUMBER];
    int subdev_counter;
    struct v4l2_async_notifier notifier;
};

static struct acamera_v4l2_subdev_t g_subdevs;


void *acamera_camera_v4l2_get_subdev_by_name( const char *name )
{
    int idx = 0;
    void *result = NULL;
    LOG( LOG_ERR, "Requested a pointer to the subdev with a name %s", name );
    for ( idx = 0; idx < V4L2_SOC_SUBDEV_NUMBER; idx++ ) {
        if ( g_subdevs.soc_subdevs[idx] && strcmp( g_subdevs.soc_subdevs[idx]->name, name ) == 0 ) {
            result = g_subdevs.soc_subdevs[idx];
            break;
        }
    }
    LOG( LOG_ERR, "Return subdev pointer 0x%x", result );
    return result;
}

int acamera_camera_v4l2_get_index_by_name( const char *name )
{
    int idx = 0;
    LOG( LOG_ERR, "Requested a index pointer with a name %s", name );
    for ( idx = 0; idx < V4L2_SOC_SUBDEV_NUMBER; idx++ ) {
        if ( g_subdevs.soc_subdevs[idx] && strcmp( g_subdevs.soc_subdevs[idx]->name, name ) == 0 ) {
            break;
        }
    }
    LOG( LOG_ERR, "Return index pointer 0x%x", idx );
    return idx;
}

static int acamera_camera_async_bound( struct v4l2_async_notifier *notifier,
                                       struct v4l2_subdev *sd,
                                       struct v4l2_async_subdev *asd )
{
    int rc = 0;
    LOG( LOG_ERR, "bound called with sd 0x%x, asd 0x%x, sd->dev 0x%x, name %s", sd, asd, sd->dev, sd->name );
    int idx = 0;
    for ( idx = 0; idx < V4L2_SOC_SUBDEV_NUMBER; idx++ ) {
        if ( g_subdevs.soc_subdevs[idx] == 0 ) {
            break;
        }
    }

    if ( idx < V4L2_SOC_SUBDEV_NUMBER ) {
        g_subdevs.soc_subdevs[idx] = sd;
        g_subdevs.subdev_counter++;

        if ( strcmp( g_subdevs.soc_subdevs[idx]->name, V4L2_SOC_IQ_NAME ) == 0 && v4l2devs_running == 1 ) { //update calibration
            fw_calibration_update();
        }

    } else {
        rc = -1;
        LOG( LOG_CRIT, "Inserted more subdevices than expected. Driver is configured to support %d subdevs only", V4L2_SOC_SUBDEV_NUMBER );
    }


    return rc;
}


static void acamera_camera_async_unbind( struct v4l2_async_notifier *notifier,
                                         struct v4l2_subdev *sd,
                                         struct v4l2_async_subdev *asd )
{
    LOG( LOG_ERR, "unbind called for subdevice sd 0x%x, asd 0x%x, sd->dev 0x%x, name %s", sd, asd, sd->dev, sd->name );

    int idx = acamera_camera_v4l2_get_index_by_name( sd->name );

    if ( strcmp( g_subdevs.soc_subdevs[idx]->name, V4L2_SOC_IQ_NAME ) != 0 ) { //any other subdevs need to stop firmware
        if ( v4l2devs_running == 1 ) {
            LOG( LOG_ERR, "stopping V4L2 firmware" );
            isp_v4l2_destroy_instance(isp_pdev);
            initialized = 0;
            v4l2devs_running = 0;
        }
    }

    if ( idx < V4L2_SOC_SUBDEV_NUMBER ) {
        g_subdevs.soc_subdevs[idx] = 0;
        g_subdevs.subdev_counter--;
    }
}


static int acamera_camera_async_complete( struct v4l2_async_notifier *notifier )
{
    int rc = 0;

    LOG( LOG_ERR, "complete called" );
    if ( v4l2devs_running == 0 ) {
        LOG( LOG_ERR, "starting V4L2 firmware" );
        rc = v4l2_device_register_subdev_nodes( &v4l2_dev );

        if ( rc == 0 ) {
            rc = isp_v4l2_create_instance( &v4l2_dev, isp_pdev);

            if ( rc == 0 ) {
                initialized = 1;
            } else {
                LOG( LOG_ERR, "Failed to register ISP v4l2 driver." );
                initialized = 0;
                rc = -1;
            }
        } else {
            rc = -1;
            LOG( LOG_CRIT, "Failed to create subdevice nodes under /dev/v4l-subdevX" );
        }
        v4l2devs_running = 1;
    }
    return rc;
}

#endif

int  write_to_file (char *fname, char *buf, int size)
{
    int ret = 0;
    struct file *fp = NULL;
    mm_segment_t old_fs;
    loff_t pos = 0;
    int nwrite = 0;
    int offset = 0;
    int first_flag=0;

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    pr_err("%s: buff addr %p, size 0x%x\n",__func__, buf, size);

    if (first_flag == 0) {
        first_flag = 1;
    /* open file to write */
        fp = filp_open(fname, O_RDWR | O_CREAT, 0666);
        if (!fp) {
            pr_err("%s: open file error\n", __func__);
            ret = -1;
            goto exit;
        }
    }

    pos = (unsigned long)offset;

	/* Write buf to file */
    nwrite=vfs_write(fp, buf, size, &pos);
    offset +=nwrite;

    if (fp) {
        filp_close(fp, NULL);
    }

exit:
    return 0;
}

#if ISP_HAS_DS1
extern uint8_t *ds1_isp_kaddr;
extern void config_ds_setting(void);
#endif
static ssize_t isp_reg_read(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
#if ISP_HAS_DS1
    config_ds_setting();
    mdelay(200);
#endif

    write_to_file("/media/lk-a.raw", isp_kaddr, 0x1fa4000);
    write_to_file("/media/lk-b.raw", isp_kaddr + 0x1fa4000, 0x1fa4000);

#if ISP_HAS_DS1
    write_to_file("/media/lk-ds1-a.raw", ds1_isp_kaddr, 0x7e9000);
    write_to_file("/media/lk-ds1-b.raw", ds1_isp_kaddr + (0x7e9000 << 1), 0x7e9000);
#endif

    LOG( LOG_ERR, "LIKE-read\n" );
    return 0;
}

static ssize_t isp_reg_write(struct device *dev,
                             struct device_attribute *attr, char const *buf, size_t size)
{
    unsigned long reg_addr = 0;
    unsigned long reg_count = 0;
    unsigned long reg_val = 0;
    unsigned long tmp_addr = 0;
    uint32_t data = 0;
    int retval = 0;
    int i = 0;
    int j = 0;
    char addr[9] = {'\0'};
    char count[16] = {'\0'};
    char val[16] = {'\0'};
    char *tmp = NULL;

    tmp = addr;

    for (i = 0; i < size; i++) {
        if (buf[i] == ' ') {
            data++;
            if (data == 1) {
                tmp = count;
                j = 0;
                continue;
            } else if (data == 2) {
                tmp = val;
                j = 0;
                continue;
            }
        }
        tmp[j] = buf[i];
        j++;
    }

    pr_err("=============================\n");

    retval = kstrtoul(addr, 16, &reg_addr);
    if (retval) {
        LOG(LOG_ERR, "Error to addr strtoul\n");
        return retval;
    }

    retval = kstrtoul(count, 10, &reg_count);
    if (retval) {
        LOG(LOG_ERR, "Error to count strtoul\n");
        return retval;
    }

    if (val[0] != '\0') {
        retval = kstrtoul(val, 16, &reg_val);
        if (retval) {
            LOG(LOG_ERR, "Error to val strtoul\n");
            return retval;
        }
    }

    if (reg_count < 1) {
        system_hw_write_32(reg_addr, reg_val);
        pr_err("Write reg 0x%08lx : 0x%08lx\n", reg_addr, reg_val);
        return size;
    }

    for (i = 0; i < reg_count; i++) {
        tmp_addr = reg_addr + (i * 4);
        data = system_hw_read_32(tmp_addr);
        pr_err("Reg 0x%08lx : 0x%08x\n", tmp_addr, data);
    }

    return size;
}

uint32_t write_reg(uint32_t val, unsigned long addr)
{
    void __iomem *io_addr;
    io_addr = ioremap_nocache(addr, 8);
    if (io_addr == NULL) {
        LOG(LOG_ERR, "%s: Failed to ioremap addr\n", __func__);
        return -1;
    }
    __raw_writel(val, io_addr);
    iounmap(io_addr);
    return 0;
}

uint32_t read_reg(unsigned long addr)
{
    void __iomem *io_addr;
    uint32_t ret;

    io_addr = ioremap_nocache(addr, 8);
    if (io_addr == NULL) {
        LOG(LOG_ERR, "%s: Failed to ioremap addr\n", __func__);
        return -1;
    }
    ret = (uint32_t)__raw_readl(io_addr);
    iounmap(io_addr);
    return ret;
}

uint32_t isp_power_on(void)
{
    uint32_t orig, tmp;

    orig = read_reg(AO_RTI_GEN_PWR_SLEEP0);			//AO_PWR_SLEEP0
    tmp = orig & 0xfff3ffff;						//set bit[18-19]=0
    write_reg(tmp, AO_RTI_GEN_PWR_SLEEP0);
    mdelay(5);
    orig = read_reg(AO_RTI_GEN_PWR_ISO0);			//AO_PWR_ISO0
    tmp = orig & 0xfff3ffff;						//set bit[18-19]=0
    write_reg(tmp, AO_RTI_GEN_PWR_ISO0);

    write_reg(0x0, HHI_ISP_MEM_PD_REG0);			//MEM_PD_REG0 set 0
    write_reg(0x0, HHI_ISP_MEM_PD_REG1);			//MEM_PD_REG1 set 0
    write_reg(0x5b446585, HHI_CSI_PHY_CNTL0);		//HHI_CSI_PHY_CNTL0
    write_reg(0x803f4321, HHI_CSI_PHY_CNTL1);		//HHI_CSI_PHY_CNTL1
    return 0;
}

static DEVICE_ATTR(isp_reg, S_IRUGO | S_IWUSR, isp_reg_read, isp_reg_write);

static void hw_reset(bool reset)
{
    void __iomem *reset_addr;
    uint32_t val;

    reset_addr = ioremap_nocache(0xffd01090, 8);;//ioremap_nocache(0xffd01014, 8);
    if (reset_addr == NULL) {
        LOG(LOG_ERR, "%s: Failed to ioremap\n", __func__);
        return;
    }

    val = __raw_readl(reset_addr);
    if (reset)
        val &= ~(1 << 1);
    else
        val |= (1 <<1);
    __raw_writel(val, reset_addr);

    if (!reset && reset_addr) {
        iounmap(reset_addr);
        reset_addr = NULL;
    }

    mdelay(5);

    iounmap(reset_addr);
    if (reset)
        LOG(LOG_INFO, "%s:reset isp\n", __func__);
    else
        LOG(LOG_INFO, "%s:release reset isp\n", __func__);
}

static int32_t isp_platform_probe( struct platform_device *pdev )
{
    int32_t rc = 0;
    struct resource *isp_res;
    struct clk* clk_isp_0;
    struct clk* clk_mipi_0;
    u32 isp_clk_rate = 666666667;
    u32 isp_mipi_rate = 200000000;

    // Initialize irq
    isp_res = platform_get_resource_byname( pdev,
        IORESOURCE_IRQ, "ISP" );

    if ( isp_res ) {
        LOG( LOG_ERR, "Juno isp irq = %d, flags = 0x%x !\n", (int)isp_res->start, (int)isp_res->flags );
        system_interrupts_set_irq( isp_res->start, isp_res->flags );
    } else {
        LOG( LOG_ERR, "Error, no isp_irq found from DT\n" );
        return -1;
    }

    isp_res = platform_get_resource( pdev,
        IORESOURCE_MEM, 0 );
    if ( isp_res ) {
        LOG( LOG_ERR, "Juno isp address = 0x%lx, end = 0x%lx !\n", isp_res->start, isp_res->end );
        if ( init_hw_io( isp_res->start, ( isp_res->end - isp_res->start ) + 1 ) != 0 ) {
            LOG( LOG_ERR, "Error on mapping gdc memory! \n" );
        }
    } else {
        LOG( LOG_ERR, "Error, no IORESOURCE_MEM DT!\n" );
    }

    isp_power_on();

    of_reserved_mem_device_init(&(pdev->dev));


    clk_isp_0 = devm_clk_get(&pdev->dev, "cts_mipi_isp_clk_composite");
    if (IS_ERR(clk_isp_0)) {
        LOG(LOG_ERR, "cannot get clock\n");
        clk_isp_0 = NULL;
        return -1;
    }
    clk_set_rate(clk_isp_0, isp_clk_rate);
    clk_prepare_enable(clk_isp_0);
    isp_clk_rate = clk_get_rate(clk_isp_0);
    LOG(LOG_ERR, "isp init clock is %d MHZ\n", isp_clk_rate / 1000000);

    clk_mipi_0 = devm_clk_get(&pdev->dev, "cts_mipi_csi_phy_clk0_composite");
    if (IS_ERR(clk_mipi_0)) {
        LOG(LOG_ERR, "cannot get clock\n");
        clk_mipi_0 = NULL;
        return -1;
    }
    clk_set_rate(clk_mipi_0, isp_mipi_rate);
    clk_prepare_enable(clk_mipi_0);
    isp_mipi_rate = clk_get_rate(clk_mipi_0);
    LOG(LOG_ERR, "mipi init clock is %d MHZ\n",isp_mipi_rate/1000000);

    hw_reset(true);

    system_interrupts_init();

    hw_reset(false);

    isp_pdev = pdev;
    static atomic_t drv_instance = ATOMIC_INIT( 0 );
    v4l2_device_set_name( &v4l2_dev, ISP_V4L2_MODULE_NAME, &drv_instance );
    rc = v4l2_device_register( NULL, &v4l2_dev );
    if ( rc == 0 ) {
        LOG( LOG_ERR, "register v4l2 driver. result %d.", rc );
    } else {
        LOG( LOG_ERR, "failed to register v4l2 device. rc = %d", rc );
        goto free_res;
    }

#if V4L2_SOC_SUBDEV_ENABLE
    int idx;

    LOG( LOG_ERR, "--------------------------------" );
    LOG( LOG_ERR, "Register %d subdevices", V4L2_SOC_SUBDEV_NUMBER );
    LOG( LOG_ERR, "--------------------------------" );

    g_subdevs.subdev_counter = 0;

    for ( idx = 0; idx < V4L2_SOC_SUBDEV_NUMBER; idx++ ) {
        g_subdevs.soc_async_sd[idx].match_type = V4L2_ASYNC_MATCH_CUSTOM;
        g_subdevs.soc_async_sd[idx].match.custom.match = NULL;

        g_subdevs.soc_async_sd_ptr[idx] = &g_subdevs.soc_async_sd[idx];
        g_subdevs.soc_subdevs[idx] = 0;
    }

    g_subdevs.notifier.bound = acamera_camera_async_bound;
    g_subdevs.notifier.complete = acamera_camera_async_complete;
    g_subdevs.notifier.unbind = acamera_camera_async_unbind;

    g_subdevs.notifier.subdevs = (struct v4l2_async_subdev **)&g_subdevs.soc_async_sd_ptr;
    g_subdevs.notifier.num_subdevs = V4L2_SOC_SUBDEV_NUMBER;

    rc = v4l2_async_notifier_register( &v4l2_dev, &g_subdevs.notifier );

    device_create_file(&pdev->dev, &dev_attr_isp_reg);
    LOG( LOG_ERR, "Init finished. async register notifier result %d. Waiting for subdevices", rc );
#else
    // no subdevice is used
    rc = isp_v4l2_create_instance( &v4l2_dev, isp_pdev);
    if ( rc < 0 )
        goto free_res;

    if ( rc == 0 ) {
        initialized = 1;
    } else {
        LOG( LOG_ERR, "Failed to register ISP v4l2 driver." );
        return -1;
    }

#endif

free_res:

    return rc;
}

static int isp_platform_remove(struct platform_device *pdev)
{
    return 0;
}

static const struct of_device_id isp_dt_match[] = {
    {.compatible = "arm, isp"},
    {}};

MODULE_DEVICE_TABLE( of, isp_dt_match );

static struct platform_driver isp_platform_driver = {
    .probe = isp_platform_probe,
    .remove	= isp_platform_remove,
    .driver = {
        .name = "arm_isp",
        .owner = THIS_MODULE,
        .of_match_table = isp_dt_match,
    },
};

static int __init fw_module_init( void )
{
    int32_t rc = 0;

    LOG( LOG_ERR, "Juno isp fw_module_init\n" );

	rc = platform_driver_register(&isp_platform_driver);

    return rc;
}

static void __exit fw_module_exit( void )
{
    LOG( LOG_ERR, "Juno isp fw_module_exit\n" );

    if ( initialized == 1 ) {
        isp_v4l2_destroy_instance(isp_pdev);
        initialized = 0;
    }

#if V4L2_SOC_SUBDEV_ENABLE
    v4l2_async_notifier_unregister( &g_subdevs.notifier );
#endif
    v4l2_device_unregister( &v4l2_dev );
    close_hw_io();
    platform_driver_unregister( &isp_platform_driver );
}

module_init( fw_module_init );
module_exit( fw_module_exit );
MODULE_LICENSE( "GPL v2" );
MODULE_AUTHOR( "ARM IVG SW" );
