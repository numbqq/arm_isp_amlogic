/*
 * videobuf2-vmalloc.c - vmalloc memory allocator for videobuf2
 *
 * Copyright (C) 2010 Samsung Electronics
 *
 * Author: Pawel Osciak <pawel@osciak.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>
#include <linux/dma-contiguous.h>

#include "isp-vb2-cmalloc.h"

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
        pr_err("Failed to alloc cma pages.\n");
        return NULL;
    }

    vaddr = phys_to_virt(paddr);

    return vaddr;
}

static void cma_free(void *buf_priv)
{
    struct vb2_cmalloc_buf *buf = buf_priv;
    struct page *cma_pages = NULL;
    struct device *dev = NULL;
    bool rc = -1;

    dev = (void *)(buf->dbuf);

    cma_pages = virt_to_page(buf->vaddr);

    rc = dma_release_from_contiguous(dev, cma_pages,
                buf->size >> PAGE_SHIFT);
    if (rc == false) {
        pr_err("Failed to release cma buffer\n");
        return;
    }

    buf->vaddr = NULL;
}


static void vb2_cmalloc_put(void *buf_priv)
{
	struct vb2_cmalloc_buf *buf = buf_priv;

	if (atomic_dec_and_test(&buf->refcount)) {
		cma_free(buf_priv);
		kfree(buf);
	}
}

static void *vb2_cmalloc_alloc(struct device *dev, unsigned long attrs,
			       unsigned long size, enum dma_data_direction dma_dir,
			       gfp_t gfp_flags)
{
	struct vb2_cmalloc_buf *buf;

	buf = kzalloc(sizeof(*buf), GFP_KERNEL | gfp_flags);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	buf->size = PAGE_ALIGN(size);
	buf->vaddr = cma_alloc(dev, buf->size);
	buf->dma_dir = dma_dir;
	buf->handler.refcount = &buf->refcount;
	buf->handler.put = vb2_cmalloc_put;
	buf->handler.arg = buf;
	buf->dbuf = (void *)dev;

	if (!buf->vaddr) {
		pr_err("cmalloc of size %ld failed\n", buf->size);
		kfree(buf);
		return ERR_PTR(-ENOMEM);
	}

	atomic_inc(&buf->refcount);
	return buf;
}

static void *vb2_cmalloc_get_userptr(struct device *dev, unsigned long vaddr,
				     unsigned long size,
				     enum dma_data_direction dma_dir)
{
	struct vb2_cmalloc_buf *buf;
	struct frame_vector *vec;
	int n_pages, offset, i;
	int ret = -ENOMEM;

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	buf->dma_dir = dma_dir;
	offset = vaddr & ~PAGE_MASK;
	buf->size = size;
	vec = vb2_create_framevec(vaddr, size, dma_dir == DMA_FROM_DEVICE);
	if (IS_ERR(vec)) {
		ret = PTR_ERR(vec);
		goto fail_pfnvec_create;
	}
	buf->vec = vec;
	n_pages = frame_vector_count(vec);
	if (frame_vector_to_pages(vec) < 0) {
		unsigned long *nums = frame_vector_pfns(vec);

		/*
		 * We cannot get page pointers for these pfns. Check memory is
		 * physically contiguous and use direct mapping.
		 */
		for (i = 1; i < n_pages; i++)
			if (nums[i-1] + 1 != nums[i])
				goto fail_map;
		buf->vaddr = (__force void *)
				ioremap_nocache(nums[0] << PAGE_SHIFT, size);
	} else {
		buf->vaddr = vm_map_ram(frame_vector_pages(vec), n_pages, -1,
					PAGE_KERNEL);
	}

	if (!buf->vaddr)
		goto fail_map;
	buf->vaddr += offset;
	return buf;

fail_map:
	vb2_destroy_framevec(vec);
fail_pfnvec_create:
	kfree(buf);

	return ERR_PTR(ret);
}

static void vb2_cmalloc_put_userptr(void *buf_priv)
{
	struct vb2_cmalloc_buf *buf = buf_priv;
	unsigned long vaddr = (unsigned long)buf->vaddr & PAGE_MASK;
	unsigned int i;
	struct page **pages;
	unsigned int n_pages;

	if (!buf->vec->is_pfns) {
		n_pages = frame_vector_count(buf->vec);
		pages = frame_vector_pages(buf->vec);
		if (vaddr)
			vm_unmap_ram((void *)vaddr, n_pages);
		if (buf->dma_dir == DMA_FROM_DEVICE)
			for (i = 0; i < n_pages; i++)
				set_page_dirty_lock(pages[i]);
	} else {
		iounmap((__force void __iomem *)buf->vaddr);
	}
	vb2_destroy_framevec(buf->vec);
	kfree(buf);
}

static void *vb2_cmalloc_vaddr(void *buf_priv)
{
	struct vb2_cmalloc_buf *buf = buf_priv;

	if (!buf->vaddr) {
		pr_err("Address of an unallocated plane requested "
		       "or cannot map user pointer\n");
		return NULL;
	}

	return buf->vaddr;
}

static unsigned int vb2_cmalloc_num_users(void *buf_priv)
{
	struct vb2_cmalloc_buf *buf = buf_priv;
	return atomic_read(&buf->refcount);
}

static int vb2_cmalloc_mmap(void *buf_priv, struct vm_area_struct *vma)
{
	struct vb2_cmalloc_buf *buf = buf_priv;
	unsigned long pfn = 0;
	unsigned long vsize = vma->vm_end - vma->vm_start;
	int ret = -1;

	if (!buf || !vma) {
		pr_err("No memory to map\n");
		return -EINVAL;
	}

	pfn = virt_to_phys(buf->vaddr) >> PAGE_SHIFT;
	ret = remap_pfn_range(vma, vma->vm_start, pfn, vsize, vma->vm_page_prot);

	if (ret) {
		pr_err("Remapping vmalloc memory, error: %d\n", ret);
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
	vma->vm_ops = &vb2_common_vm_ops;

	vma->vm_ops->open(vma);

	return 0;
}

const struct vb2_mem_ops vb2_cmalloc_memops = {
	.alloc		= vb2_cmalloc_alloc,
	.put		= vb2_cmalloc_put,
	.get_userptr	= vb2_cmalloc_get_userptr,
	.put_userptr	= vb2_cmalloc_put_userptr,
#ifdef CONFIG_HAS_DMA
	.get_dmabuf	=  NULL,
#endif
	.map_dmabuf	= NULL,
	.unmap_dmabuf	= NULL,
	.attach_dmabuf	= NULL,
	.detach_dmabuf	= NULL,
	.vaddr		= vb2_cmalloc_vaddr,
	.mmap		= vb2_cmalloc_mmap,
	.num_users	= vb2_cmalloc_num_users,
};
EXPORT_SYMBOL_GPL(vb2_cmalloc_memops);

MODULE_DESCRIPTION("cmalloc memory handling routines for videobuf2");
MODULE_AUTHOR("Keke Li<keke.li@amlogic.com>");
MODULE_LICENSE("GPL");
