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

#include <linux/videodev2.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-vmalloc.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-memops.h>

#include <linux/dma-mapping.h>
#include <linux/dma-contiguous.h>

#include "acamera_logger.h"
#include "isp-v4l2-common.h"
#include "isp-v4l2-stream.h"
#include "isp-vb2.h"

struct vb2_vmalloc_buf {
    void *vaddr;
    struct frame_vector	*vec;
    enum dma_data_direction	dma_dir;
    unsigned long size;
    atomic_t refcount;
    struct vb2_vmarea_handler handler;
    struct dma_buf *dbuf;
};

static struct vb2_mem_ops isp_vb2_memops;

/* ----------------------------------------------------------------
 * VB2 operations
 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 0))
static int isp_vb2_queue_setup( struct vb2_queue *vq,
                                unsigned int *nbuffers, unsigned int *nplanes,
                                unsigned int sizes[], struct device *alloc_devs[] )
#else
static int isp_vb2_queue_setup( struct vb2_queue *vq, const struct v4l2_format *fmt,
                                unsigned int *nbuffers, unsigned int *nplanes,
                                unsigned int sizes[], void *alloc_ctxs[] )
#endif
{
    static unsigned long cnt = 0;
    isp_v4l2_stream_t *pstream = vb2_get_drv_priv( vq );
    struct v4l2_format vfmt;
    //unsigned int size;
    int i;
    LOG( LOG_INFO, "Enter id:%d, cnt: %lu.", pstream->stream_id, cnt++ );
    LOG( LOG_INFO, "vq: %p, *nplanes: %u.", vq, *nplanes );

    // get current format
    if ( isp_v4l2_stream_get_format( pstream, &vfmt ) < 0 ) {
        LOG( LOG_ERR, "fail to get format from stream" );
        return -EBUSY;
    }

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 8, 0))
    if ( fmt )
        LOG( LOG_INFO, "fmt: %p, width: %u, height: %u, sizeimage: %u.",
             fmt, fmt->fmt.pix.width, fmt->fmt.pix.height, fmt->fmt.pix.sizeimage );
#endif

    LOG( LOG_INFO, "vq->num_buffers: %u vq->type:%u.", vq->num_buffers, vq->type );
    if ( vq->num_buffers + *nbuffers < 3 )
        *nbuffers = 3 - vq->num_buffers;

    if ( vfmt.type == V4L2_BUF_TYPE_VIDEO_CAPTURE ) {
        *nplanes = 1;
        sizes[0] = vfmt.fmt.pix.sizeimage;
        LOG( LOG_INFO, "nplanes: %u, size: %u", *nplanes, sizes[0] );
    } else if ( vfmt.type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE ) {
        *nplanes = vfmt.fmt.pix_mp.num_planes;
        LOG( LOG_INFO, "nplanes: %u", *nplanes );
        for ( i = 0; i < vfmt.fmt.pix_mp.num_planes; i++ ) {
            sizes[i] = vfmt.fmt.pix_mp.plane_fmt[i].sizeimage;
            LOG( LOG_INFO, "size: %u", sizes[i] );
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 8, 0))
            alloc_ctxs[i] = 0;
#endif
        }
    } else {
        LOG( LOG_ERR, "Wrong type: %u", vfmt.type );
        return -EINVAL;
    }

    /*
     * videobuf2-vmalloc allocator is context-less so no need to set
     * alloc_ctxs array.
     */

    LOG( LOG_INFO, "nbuffers: %u, nplanes: %u", *nbuffers, *nplanes );

    return 0;
}

static int isp_vb2_buf_prepare( struct vb2_buffer *vb )
{
    unsigned long size;
    isp_v4l2_stream_t *pstream = vb2_get_drv_priv( vb->vb2_queue );
    static unsigned long cnt = 0;
    struct v4l2_format vfmt;
    int i;
    LOG( LOG_INFO, "Enter id:%d, cnt: %lu.", pstream->stream_id, cnt++ );

    // get current format
    if ( isp_v4l2_stream_get_format( pstream, &vfmt ) < 0 ) {
        LOG( LOG_ERR, "fail to get format from stream" );
        return -EBUSY;
    }

    if ( vfmt.type == V4L2_BUF_TYPE_VIDEO_CAPTURE ) {
        size = vfmt.fmt.pix.sizeimage;

        if ( vb2_plane_size( vb, 0 ) < size ) {
            LOG( LOG_ERR, "buffer too small (%lu < %lu)", vb2_plane_size( vb, 0 ), size );
            return -EINVAL;
        }

        vb2_set_plane_payload( vb, 0, size );
        LOG( LOG_INFO, "single plane payload set %d", size );
    } else if ( vfmt.type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE ) {
        for ( i = 0; i < vfmt.fmt.pix_mp.num_planes; i++ ) {
            size = vfmt.fmt.pix_mp.plane_fmt[i].sizeimage;
            if ( vb2_plane_size( vb, i ) < size ) {
                LOG( LOG_ERR, "i:%d buffer too small (%lu < %lu)", i, vb2_plane_size( vb, 0 ), size );
                return -EINVAL;
            }
            vb2_set_plane_payload( vb, i, size );
            LOG( LOG_INFO, "i:%d payload set %d", i, size );
        }
    }

    return 0;
}

static int isp_vb_to_tframe(tframe_t *frame, isp_v4l2_buffer_t *buf)
{
    void *p_mem = NULL;
    void *s_mem = NULL;
    unsigned int p_size = 0;
    unsigned int s_size = 0;

    if (frame == NULL || buf == NULL) {
        LOG(LOG_ERR, "Error input param");
        return -1;
    }

    p_mem = vb2_plane_vaddr(&buf->vvb.vb2_buf, 0);
    p_size = PAGE_ALIGN(buf->vvb.vb2_buf.planes[0].length);

    s_mem = vb2_plane_vaddr(&buf->vvb.vb2_buf, 1);
    s_size = PAGE_ALIGN(buf->vvb.vb2_buf.planes[1].length);

    frame->primary.address = virt_to_phys(p_mem);
    frame->primary.size = p_size;
    frame->secondary.address = virt_to_phys(s_mem);
    frame->secondary.size = s_size;
    frame->list = (void *)&buf->list;

    return 0;
}

static void isp_frame_buff_queue(void *stream, isp_v4l2_buffer_t *buf)
{
    isp_v4l2_stream_t *pstream = NULL;
    int32_t s_type = -1;
    uint8_t d_type = 0;
    uint32_t rtn = 0;
    tframe_t f_buff;
    int rc = -1;

    if (stream == NULL || buf == NULL) {
        LOG(LOG_ERR, "Error input param\n");
        return;
    }

    pstream = stream;
    memset(&f_buff, 0, sizeof(f_buff));

    s_type = pstream->stream_type;

    switch (s_type) {
    case V4L2_STREAM_TYPE_FR:
        d_type = dma_fr;
        break;
    case V4L2_STREAM_TYPE_DS1:
        d_type = dma_ds1;
        break;
    default:
        return;
    }

    rc = isp_vb_to_tframe(&f_buff, buf);
    if (rc != 0)
        return;

    acamera_api_dma_buffer(d_type, &f_buff, 1, &rtn);
}

static void isp_vb2_buf_queue( struct vb2_buffer *vb )
{
    isp_v4l2_stream_t *pstream = vb2_get_drv_priv( vb->vb2_queue );
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
    struct vb2_v4l2_buffer *vvb = to_vb2_v4l2_buffer( vb );
    isp_v4l2_buffer_t *buf = container_of( vvb, isp_v4l2_buffer_t, vvb );
#else
    isp_v4l2_buffer_t *buf = container_of( vb, isp_v4l2_buffer_t, vb );
#endif
    static unsigned long cnt = 0;

    LOG( LOG_INFO, "Enter id:%d, cnt: %lu.", pstream->stream_id, cnt++ );

    spin_lock( &pstream->slock );
    list_add_tail( &buf->list, &pstream->stream_buffer_list );
    isp_frame_buff_queue(pstream, buf);
    spin_unlock( &pstream->slock );
}

static const struct vb2_ops isp_vb2_ops = {
    .queue_setup = isp_vb2_queue_setup, // called from VIDIOC_REQBUFS
    .buf_prepare = isp_vb2_buf_prepare,
    .buf_queue = isp_vb2_buf_queue,
    .wait_prepare = vb2_ops_wait_prepare,
    .wait_finish = vb2_ops_wait_finish,
};

static void *cma_alloc(struct device *dev, unsigned long size)
{
    struct page *cma_pages = NULL;
    dma_addr_t paddr = 0;
    void *vaddr = NULL;

    cma_pages = dma_alloc_from_contiguous(dev,
            size >> PAGE_SHIFT, 0);
    if (cma_pages) {
        paddr = page_to_phys(cma_pages);
    } else {
        LOG(LOG_ERR, "Failed to alloc cma pages.\n");
        return NULL;
    }

    vaddr = phys_to_virt(paddr);

    return vaddr;
}

static void cma_free(void *buf_priv)
{
    struct vb2_vmalloc_buf *buf = buf_priv;
    struct page *cma_pages = NULL;
    struct device *dev = NULL;
    bool rc = -1;

    dev = (void *)(buf->dbuf);

    cma_pages = virt_to_page(buf->vaddr);

    rc = dma_release_from_contiguous(dev, cma_pages,
                buf->size >> PAGE_SHIFT);
    if (rc == false) {
        LOG(LOG_ERR, "Failed to release cma buffer\n");
        return;
    }

    buf->vaddr = NULL;
}

static void vb2_cma_put(void *buf_priv)
{
    struct vb2_vmalloc_buf *buf = buf_priv;

    if (atomic_dec_and_test(&buf->refcount)) {
        cma_free(buf_priv);
        kfree(buf);
    }
}

static void *vb2_cma_alloc(struct device *dev, unsigned long attrs,
                    unsigned long size, enum dma_data_direction dma_dir,
                    gfp_t gfp_flags)
{
    struct vb2_vmalloc_buf *buf;

    buf = kzalloc(sizeof(*buf), GFP_KERNEL | gfp_flags);
    if (!buf)
        return ERR_PTR(-ENOMEM);

    buf->size = size;
    buf->vaddr = cma_alloc(dev, buf->size);
    buf->dma_dir = dma_dir;
    buf->handler.refcount = &buf->refcount;
    buf->handler.put = vb2_cma_put;
    buf->handler.arg = buf;
    buf->dbuf = (void *)dev;

    if (!buf->vaddr) {
        LOG(LOG_ERR, "vmalloc of size %ld failed\n", buf->size);
        kfree(buf);
        return ERR_PTR(-ENOMEM);
    }

    atomic_inc(&buf->refcount);
    return buf;
}

static int vb2_cma_mmap(void *buf_priv, struct vm_area_struct *vma)
{
    struct vb2_vmalloc_buf *buf = buf_priv;
    unsigned long pfn = 0;
    unsigned long vsize = vma->vm_end - vma->vm_start;
    int ret = -1;

    if (!buf || !vma) {
        LOG(LOG_ERR, "No memory to map\n");
        return -EINVAL;
    }

    pfn = virt_to_phys(buf->vaddr) >> PAGE_SHIFT;
    ret = remap_pfn_range(vma, vma->vm_start, pfn, vsize, vma->vm_page_prot);

    if (ret) {
        LOG(LOG_ERR, "Remapping vmalloc memory, error: %d\n", ret);
        return ret;
    }

    /*
    * Make sure that vm_areas for 2 buffers won't be merged together
    */
    vma->vm_flags |= VM_DONTEXPAND;

    /*
    * Use common vm_area operations to track buffer refcount.
    */
    vma->vm_private_data = &buf->handler;
    vma->vm_ops	= &vb2_common_vm_ops;

    vma->vm_ops->open(vma);

    return 0;
}


/* ----------------------------------------------------------------
 * VB2 external interface for isp-v4l2
 */
int isp_vb2_queue_init( struct vb2_queue *q, struct mutex *mlock, isp_v4l2_stream_t *pstream )
{
    memset( q, 0, sizeof( struct vb2_queue ) );

    /* start creating the vb2 queues */

    //all stream multiplanar
    q->type = pstream->cur_v4l2_fmt.type;

    LOG( LOG_INFO, "vb2 init for stream:%d type: %u.", pstream->stream_id, q->type );

    if (pstream->stream_id == V4L2_STREAM_TYPE_FR ||
            pstream->stream_id == V4L2_STREAM_TYPE_DS1) {
        memset(&isp_vb2_memops, 0, sizeof(isp_vb2_memops));
        isp_vb2_memops = vb2_vmalloc_memops;

        isp_vb2_memops.alloc = vb2_cma_alloc;
        isp_vb2_memops.put = vb2_cma_put;
        isp_vb2_memops.mmap = vb2_cma_mmap;

        q->mem_ops = &isp_vb2_memops;
    } else {
        q->mem_ops = &vb2_vmalloc_memops;
    }

    q->io_modes = VB2_MMAP | VB2_READ;
    q->drv_priv = pstream;
    q->buf_struct_size = sizeof( isp_v4l2_buffer_t );

    q->ops = &isp_vb2_ops;
    q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    q->min_buffers_needed = 3;
    q->lock = mlock;

    return vb2_queue_init( q );
}

void isp_vb2_queue_release( struct vb2_queue *q )
{
    vb2_queue_release( q );
}
