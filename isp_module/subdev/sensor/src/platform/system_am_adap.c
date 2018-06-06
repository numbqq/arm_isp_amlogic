/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2018 Amlogic or its affiliates
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
#define pr_fmt(fmt) "AM_ADAP: " fmt

#include "system_am_adap.h"
#include <linux/irqreturn.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/ioport.h>
#include <linux/of_platform.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/dma-contiguous.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_fdt.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/io.h>


#define AM_ADAPTER_NAME "amlogic, isp-adapter"


struct am_adap *g_adap = NULL;
struct am_adap_info para;

/*we allocte from CMA*/
static uint8_t *isp_cma_mem = NULL;
static struct page *cma_pages = NULL;
static resource_size_t buffer_start;

#define CMA_ALLOC_SIZE 16

static resource_size_t ddr_buf[2];


static int ft_buf_index;
static int rd_buf_index;
static int dump_flag;

int  write_to_file (char *buf, int size)
{
	int ret = 0;
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos = 0;
	int nwrite = 0;
	static int offset = 0;
	static int first_flag=0;

	/* change to KERNEL_DS address limit */
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (first_flag == 0) {
		first_flag=1;
		/* open file to write */
		fp = filp_open("/media/adapt_img.raw", O_WRONLY|O_CREAT, 0640);
		if (!fp) {
		   printk("%s: open file error\n", __FUNCTION__);
		   ret = -1;
		   goto exit;
		}
	}
	pos=(unsigned long)offset;

	/* Write buf to file */
	nwrite=vfs_write(fp, buf, size, &pos);
	offset +=nwrite;

	if (fp) {
		filp_close(fp, NULL);
	}

exit:
		return ret;
}

static ssize_t adapt_frame_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	pr_info("adapt-read.\n");
	uint8_t buf1[100];
	buf = phys_to_virt(ddr_buf[ft_buf_index]);//ft_buf_index^1
	write_to_file(buf, 2880 * 1080);
	return sprintf(buf1,"dump flag:%d", dump_flag);
}

static ssize_t adapt_frame_write(struct device *dev,
	struct device_attribute *attr, char const *buf, size_t size)
{
	unsigned long write_flag = 0;
	int retval = 0;

	retval = kstrtoul(buf, 10, &write_flag);

	if (retval) {
		pr_err("Error to count strtoul\n");
		return retval;
	}

	dump_flag = write_flag;

	return size;
}
static DEVICE_ATTR(adapt_frame, S_IRUGO | S_IWUSR, adapt_frame_read, adapt_frame_write);

int am_adap_parse_dt(struct device_node *node)
{
	int rtn = -1;
	int irq = -1;
	int ret = 0;
	struct resource rs;
	struct am_adap *t_adap = NULL;

	if (node == NULL) {
		pr_err("%s: Error input param\n", __func__);
		return -1;
	}

	rtn = of_device_is_compatible(node, AM_ADAPTER_NAME);
	if (rtn == 0) {
		pr_err("%s: Error match compatible\n", __func__);
		return -1;
	}

	t_adap = kzalloc(sizeof(*t_adap), GFP_KERNEL);
	if (t_adap == NULL) {
		pr_err("%s: Failed to alloc adapter\n", __func__);
		return -1;
	}

	t_adap->of_node = node;

	rtn = of_address_to_resource(node, 0, &rs);
	if (rtn != 0) {
		pr_err("%s:Error get adap reg resource\n", __func__);
		goto reg_error;
	}

	pr_info("%s: rs idx info: name: %s\n", __func__, rs.name);
	if (strcmp(rs.name, "adapter") == 0) {
		t_adap->reg = rs;
		t_adap->base_addr =
				ioremap_nocache(t_adap->reg.start, resource_size(&t_adap->reg));
	}

	irq = irq_of_parse_and_map(node, 0);
	if (irq <= 0) {
		pr_err("%s:Error get adap irq\n", __func__);
		goto irq_error;
	}

	t_adap->f_end_irq = irq;
	pr_info("%s:rs info: irq: %d\n", __func__, t_adap->f_end_irq);

	irq = irq_of_parse_and_map(node, 1);
	if (irq <= 0) {
		pr_err("%s:Error get adap irq\n", __func__);
		goto irq_error;
	}

	t_adap->rd_irq = irq;
	pr_info("%s:rs info: irq: %d\n", __func__, t_adap->rd_irq);

	t_adap->p_dev = of_find_device_by_node(node);
	ret = of_reserved_mem_device_init(&(t_adap->p_dev->dev));
	if (ret != 0) {
		pr_err("adapt reserved mem device init failed.\n");
		return ret;
	}

	device_create_file(&(t_adap->p_dev->dev), &dev_attr_adapt_frame);

	g_adap = t_adap;

	return 0;

irq_error:
	iounmap(t_adap->base_addr);
	t_adap->base_addr = NULL;


reg_error:
	if (t_adap != NULL)
		kfree(t_adap);
	return -1;
}

static inline void update_wr_reg_bits(unsigned int reg,
				adap_io_type_t io_type, unsigned int mask,
				unsigned int val)
{
	unsigned int tmp, orig;
	void __iomem *base = NULL;
	switch (io_type) {
		case FRONTEND_IO:
			base = g_adap->base_addr + FRONTEND_BASE;
			break;
		case RD_IO:
			base = g_adap->base_addr + RD_BASE;
			break;
		case PIXEL_IO:
			base = g_adap->base_addr + PIXEL_BASE;
			break;
		case ALIGN_IO:
			base = g_adap->base_addr + ALIGN_BASE;
			break;
		case MISC_IO:
			base = g_adap->base_addr + MISC_BASE;
		default:
			pr_err("adapter error io type.\n");
			base = NULL;
			break;
	}
	if (base !=  NULL) {
		orig = readl(base + reg);
		tmp = orig & ~mask;
		tmp |= val & mask;
		writel(tmp, base + reg);
	}
}

static inline void adap_wr_reg_bits(unsigned int adr,
	    adap_io_type_t io_type, unsigned int val,
		unsigned int start, unsigned int len)
{
	update_wr_reg_bits(adr, io_type,
		           ((1<<len)-1)<<start, val<<start);
}

static inline void mipi_adap_reg_wr(int addr, adap_io_type_t io_type, uint32_t val)
{
	void __iomem *base_reg_addr = NULL;
	void __iomem *reg_addr = NULL;
	switch (io_type) {
		case FRONTEND_IO:
			base_reg_addr = g_adap->base_addr + FRONTEND_BASE;
			break;
		case RD_IO:
			base_reg_addr = g_adap->base_addr + RD_BASE;
			break;
		case PIXEL_IO:
			base_reg_addr = g_adap->base_addr + PIXEL_BASE;
			break;
		case ALIGN_IO:
			base_reg_addr = g_adap->base_addr + ALIGN_BASE;
			break;
		case MISC_IO:
			base_reg_addr = g_adap->base_addr + MISC_BASE;
		default:
			pr_err("adapter error io type.\n");
			base_reg_addr = NULL;
			break;
	}
	if (base_reg_addr != NULL) {
		reg_addr = base_reg_addr + addr;
		writel(val, reg_addr);
	} else
		pr_err("mipi adapter write register failed.\n");

}

void am_adap_set_info(struct am_adap_info *info)
{
	memset(&para, 0, sizeof(struct am_adap_info));
	memcpy(&para, info, sizeof(struct am_adap_info));
}

int am_adap_get_depth(void)
{
	int depth = 0;
	switch (para.fmt) {
		case AM_RAW6:
			depth = 6;
			break;
		case AM_RAW7:
			depth = 7;
			break;
		case AM_RAW8:
			depth = 8;
			break;
		case AM_RAW10:
			depth = 10;
			break;
		case AM_RAW12:
			depth = 12;
			break;
		case AM_RAW14:
			depth = 14;
			break;
		default:
			pr_err("Not supported data format.\n");
	}
	return depth;
}

int am_disable_irq(void)
{
	//disable irq mask
	mipi_adap_reg_wr(CSI2_INTERRUPT_CTRL_STAT, FRONTEND_IO, 0x00000000);
	mipi_adap_reg_wr(MIPI_ADAPT_IRQ_MASK0, ALIGN_IO, 0x00000000);

	return 0;
}

/*
 *========================AM ADAPTER FRONTEND INTERFACE========================
 */

void am_adap_frontend_start(void)
{
	int width = para.img.width;
	//int height = para.img.height;
	int depth, val;
	depth = am_adap_get_depth();
	if (!depth)
		pr_err("is not supported data format.");
	adap_wr_reg_bits(CSI2_GEN_CTRL0, FRONTEND_IO, 1, 0, 1);
	pr_info("frontend_start width = %d\n", width);
	val = round_up(((((width * depth)/8)/16)), 16);
	adap_wr_reg_bits(CSI2_DDR_STRIDE_PIX, FRONTEND_IO, val, 4, 28);
}

int am_adap_frontend_init(void)
{
	mipi_adap_reg_wr(CSI2_CLK_RESET, FRONTEND_IO, 0x0);//release from reset
	mipi_adap_reg_wr(CSI2_CLK_RESET, FRONTEND_IO, 0x6);//enable frontend module clock and disable auto clock gating

	if (para.mode == DIR_MODE) {
		if (para.path == PATH0)
			mipi_adap_reg_wr(CSI2_GEN_CTRL0, FRONTEND_IO, 0x001f0001);//bit[0] 1:enable virtual channel 0
	} else if (para.mode == DDR_MODE) {
		if (para.path == PATH0)
			mipi_adap_reg_wr(CSI2_GEN_CTRL0, FRONTEND_IO, 0x001f0011);
	} else {
		pr_err("%s, Not supported Mode.\n", __func__);
	}

	//applicable only to Raw data, direct MEM path
	mipi_adap_reg_wr(CSI2_X_START_END_MEM, FRONTEND_IO, 0xffff0000);
	mipi_adap_reg_wr(CSI2_Y_START_END_MEM, FRONTEND_IO, 0xffff0000);

	if (para.mode == DDR_MODE) {
		//config ddr_buf[0] address
		adap_wr_reg_bits(CSI2_DDR_START_PIX, FRONTEND_IO, ddr_buf[ft_buf_index], 0, 32);
	}

	//set frame size
	mipi_adap_reg_wr(CSI2_DDR_STRIDE_PIX, FRONTEND_IO, 0x00000780);
	//enable vs_rise_isp interrupt & enable ddr_wdone interrupt
	mipi_adap_reg_wr(CSI2_INTERRUPT_CTRL_STAT, FRONTEND_IO, 0x5);

	return 0;
}

/*
 *========================AM ADAPTER READER INTERFACE==========================
 */

void am_adap_reader_start(void)
{
	int height = para.img.height;
	int width = para.img.width;
	int val, depth;
	depth = am_adap_get_depth();
	val = round_up(((((width * depth)/8)/16)), 16);
	adap_wr_reg_bits(MIPI_ADAPT_DDR_RD0_CNTL1, RD_IO, height, 16, 13);
	adap_wr_reg_bits(MIPI_ADAPT_DDR_RD0_CNTL1, RD_IO, val, 0, 10);
	adap_wr_reg_bits(MIPI_ADAPT_DDR_RD0_CNTL0, RD_IO, 1, 0, 1);
}

int am_adap_reader_init(void)
{
	if (para.mode == DIR_MODE) {
		mipi_adap_reg_wr(MIPI_ADAPT_DDR_RD0_CNTL1, RD_IO, 0x02d00078);
		mipi_adap_reg_wr(MIPI_ADAPT_DDR_RD0_CNTL0, RD_IO, 0xb5000005);
	} else if (para.mode == DDR_MODE) {
		mipi_adap_reg_wr(MIPI_ADAPT_DDR_RD0_CNTL1, RD_IO, 0x02d00078);
		adap_wr_reg_bits(MIPI_ADAPT_DDR_RD0_CNTL2, RD_IO, ddr_buf[rd_buf_index], 0, 32);//ddr mode config frame address
		rd_buf_index++;
		mipi_adap_reg_wr(MIPI_ADAPT_DDR_RD0_CNTL0, RD_IO, 0x70000001);
	} else {
		pr_err("%s, Not supported Mode.\n", __func__);
	}

	return 0;
}


/*
 *========================AM ADAPTER PIXEL INTERFACE===========================
 */

void am_adap_pixel_start(void)
{
	int fmt = para.fmt;
	int width = para.img.width;
	adap_wr_reg_bits(MIPI_ADAPT_PIXEL0_CNTL0, PIXEL_IO, fmt, 13, 3);
	adap_wr_reg_bits(MIPI_ADAPT_PIXEL0_CNTL0, PIXEL_IO, width, 0, 13);
	adap_wr_reg_bits(MIPI_ADAPT_PIXEL0_CNTL1, PIXEL_IO, 1, 31, 1);
}


int am_adap_pixel_init(void)
{
	if (para.mode == DIR_MODE) {
		//default width 1280
		mipi_adap_reg_wr(MIPI_ADAPT_PIXEL0_CNTL0, PIXEL_IO, 0x8000a500);
		mipi_adap_reg_wr(MIPI_ADAPT_PIXEL0_CNTL1, PIXEL_IO, 0x80000808);
	} else if (para.mode == DDR_MODE) {
		mipi_adap_reg_wr(MIPI_ADAPT_PIXEL0_CNTL0, PIXEL_IO, 0x0000a500);
		mipi_adap_reg_wr(MIPI_ADAPT_PIXEL0_CNTL1, PIXEL_IO, 0x80000008);
	} else {
		pr_err("%s, Not supported Mode.\n", __func__);
	}

	return 0;
}


/*
 *========================AM ADAPTER ALIGNMENT INTERFACE=======================
 */

void am_adap_alig_start(void)
{
	int width, height, alig_width, alig_height, val;
	width = para.img.width;
	height = para.img.height;
	alig_width = width + 40; //hblank > 32 cycles
	alig_height = height + 60; //vblank > 48 lines
	val = width + 35; // width < val < alig_width
	adap_wr_reg_bits(MIPI_ADAPT_ALIG_CNTL0, ALIGN_IO, alig_width, 0, 13);
	adap_wr_reg_bits(MIPI_ADAPT_ALIG_CNTL0, ALIGN_IO, alig_height, 16, 13);
	adap_wr_reg_bits(MIPI_ADAPT_ALIG_CNTL1, ALIGN_IO, width, 16, 13);
	adap_wr_reg_bits(MIPI_ADAPT_ALIG_CNTL2, ALIGN_IO, height, 16, 13);
	adap_wr_reg_bits(MIPI_ADAPT_ALIG_CNTL8, ALIGN_IO, val, 16, 13);
	adap_wr_reg_bits(MIPI_ADAPT_ALIG_CNTL8, ALIGN_IO, 1, 31, 1);
}


int am_adap_alig_init(void)
{
	//default width 1280, height 720
	mipi_adap_reg_wr(MIPI_ADAPT_ALIG_CNTL0, ALIGN_IO, 0x02f80528);//associate width and height
	mipi_adap_reg_wr(MIPI_ADAPT_ALIG_CNTL1, ALIGN_IO, 0x05000000);//associate width
	mipi_adap_reg_wr(MIPI_ADAPT_ALIG_CNTL2, ALIGN_IO, 0x02d00000);//associate height
	if (para.mode == DIR_MODE) {
		mipi_adap_reg_wr(MIPI_ADAPT_ALIG_CNTL6, ALIGN_IO, 0x00fff011);
		mipi_adap_reg_wr(MIPI_ADAPT_ALIG_CNTL7, ALIGN_IO, 0xc350c000);
		mipi_adap_reg_wr(MIPI_ADAPT_ALIG_CNTL8, ALIGN_IO, 0x85231020);
	} else if (para.mode == DDR_MODE) {
		mipi_adap_reg_wr(MIPI_ADAPT_ALIG_CNTL6, ALIGN_IO, 0x00fff001);
		mipi_adap_reg_wr(MIPI_ADAPT_ALIG_CNTL7, ALIGN_IO, 0x0);
		mipi_adap_reg_wr(MIPI_ADAPT_ALIG_CNTL8, ALIGN_IO, 0x80000020);
	} else {
		pr_err("Not supported mode.\n");
	}
	mipi_adap_reg_wr(MIPI_ADAPT_IRQ_MASK0, ALIGN_IO, 0x00002100);

	return 0;
}

/*
 *========================AM ADAPTER INTERFACE==========================
 */

static irqreturn_t adpapter_rd_isr(int irq, void *data)
{
	mipi_adap_reg_wr(MIPI_ADAPT_IRQ_PENDING0, ALIGN_BASE, 0xffffffff);
	return IRQ_HANDLED;
}

static irqreturn_t adpapter_wr_isr(int irq, void *data)
{
	mipi_adap_reg_wr(CSI2_INTERRUPT_CTRL_STAT, FRONTEND_IO, 0xffffffff);
	return IRQ_HANDLED;
}

int am_adap_alloc_mem(void)
{

	if (para.mode == DDR_MODE) {
		cma_pages = dma_alloc_from_contiguous(
				  &(g_adap->p_dev->dev),
				  (CMA_ALLOC_SIZE*SZ_1M) >> PAGE_SHIFT, 0);
		if (cma_pages) {
			buffer_start = page_to_phys(cma_pages);
			pr_info("adapt phy addr = %llx\n", buffer_start);
		} else {
			pr_err("alloc cma pages failed.\n");
			return 0;
		}
		isp_cma_mem = phys_to_virt(buffer_start);
		pr_info("isp_cma_mem = %p\n", isp_cma_mem);
	}
	return 0;
}

int am_adap_init(void)
{
	int ret = 0;
	int depth;
	if (para.mode == DDR_MODE) {
		am_adap_alloc_mem();
		depth = am_adap_get_depth();
		if (cma_pages) {
			//note important : ddr_buf[0] and ddr_buf[1] address should alignment 16 byte
			ddr_buf[0] = buffer_start;
			ddr_buf[1] = buffer_start + ((para.img.width) * (para.img.height) * depth)/8;
			ddr_buf[1] = (ddr_buf[1] + 15)&(~15);
		}
	}

	mipi_adap_reg_wr(CSI2_CLK_RESET, FRONTEND_IO, 1);
	mipi_adap_reg_wr(CSI2_CLK_RESET, FRONTEND_IO, 0);
	if (para.mode == DDR_MODE) {
		ret = request_irq(g_adap->f_end_irq, &adpapter_wr_isr, IRQF_SHARED | IRQF_TRIGGER_HIGH,
		"frontend-irq", (void *)g_adap);
		pr_info("zgs : frontend irq = %d, ret = %d\n", g_adap->f_end_irq, ret);
		ret = request_irq(g_adap->rd_irq, &adpapter_rd_isr, IRQF_SHARED | IRQF_TRIGGER_HIGH,
		"ddr_rd-irq", (void *)g_adap);
		pr_info("zgs : ddr_rd irq = %d, ret = %d\n", g_adap->rd_irq, ret);
	}

	ft_buf_index = 0;
	rd_buf_index = 0;

	//default setting : 720p & RAW12
	am_adap_frontend_init();
	am_adap_reader_init();
	am_adap_pixel_init();
	am_adap_alig_init();

	dump_flag = 0;
	return 0;
}

int am_adap_start(int idx)
{
	am_adap_alig_start();
	am_adap_pixel_start();
	am_adap_reader_start();
	am_adap_frontend_start();

	return 0;
}

int am_adap_free_mem(void)
{
	if (para.mode == DDR_MODE) {
		if (cma_pages) {
			dma_release_from_contiguous(
				 &(g_adap->p_dev->dev),
				 cma_pages,
				 (CMA_ALLOC_SIZE*SZ_1M) >> PAGE_SHIFT);
			cma_pages = NULL;
			buffer_start = 0;
			pr_info("release alloc CMA buffer.");
		}
	}
	return 0;
}

int am_adap_reset(void)
{
	mipi_adap_reg_wr(CSI2_CLK_RESET, FRONTEND_IO, 0x0);
	mipi_adap_reg_wr(CSI2_CLK_RESET, FRONTEND_IO, 0x6);
	mipi_adap_reg_wr(CSI2_GEN_CTRL0, FRONTEND_IO, 0x001f0000);
	mipi_adap_reg_wr(MIPI_OTHER_CNTL0, MISC_IO, 0xf0000000);
	mipi_adap_reg_wr(MIPI_OTHER_CNTL0, MISC_IO, 0x00000000);

	return 0;
}

int am_adap_deinit(void)
{
	am_adap_free_mem();
	am_adap_reset();
	am_disable_irq();
	ft_buf_index = 0;
	rd_buf_index = 0;
	dump_flag = 0;
	memset(&para, 0, sizeof(struct am_adap_info));
	return 0;
}

