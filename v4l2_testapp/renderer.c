//----------------------------------------------------------------------------
//   The confidential and proprietary information contained in this file may
//   only be used by a person authorised under and to the extent permitted
//   by a subsisting licensing agreement from ARM Limited or its affiliates.
//
//          (C) COPYRIGHT [2018] ARM Limited or its affiliates.
//              ALL RIGHTS RESERVED
//
//   This entire notice must be reproduced on all copies of this file
//   and copies of this file may only be made by a person if such person is
//   permitted to do so under the terms of a subsisting license agreement
//   from ARM Limited or its affiliates.
//----------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <linux/fb.h>
#include <linux/videodev2.h>

#include "common.h"
#include "renderer.h"

#include <unistd.h>
#include <fcntl.h>
#include <malloc.h>
#include <ge2d_port.h>
#include <aml_ge2d.h>

int renderImage(unsigned char *dst, struct fb_var_screeninfo vinfo, struct fb_fix_screeninfo finfo,
    unsigned char *src, int width, int height, render_mode_t mode, int fb_fd, int fb_index)
{
    int i, j;
    int src_x, src_y, dst_x, dst_y, w, h;
	int k;
    int pos_src = 0, pos_dst = 0;
    int bit_depth = vinfo.bits_per_pixel / 8;

    switch (mode) {
    case AFD_RENDER_MODE_CENTER:
        // Center is not supported yet !
    case AFD_RENDER_MODE_LEFT_TOP:
        src_x = 0;
        src_y = 0;
        dst_x = 0;
        dst_y = 0;
        /*
        if (vinfo.xres > src.width) w = src.width;
        else w = vinfo.xres;
        if (vinfo.yres > src.height) h = src.height;
        else h = vinfo.yres;
        */
        w = width;
        h = height;
        break;

    default:
        perror("Error, invalid mode value");
        exit(1);
        break;
    }

    /*
    if (src.fmt == ISP_V4L2_PIX_FMT_ARGB2101010) {
        // translate YUV to RGB line by line
        uint64_t *src_u64;
        uint64_t *dst_u64;
        //int32_t offset = 0;
        uint64_t value = 0;

        for (j = 0; j < h; j++) {
            pos_src = (src_y + j) * (src.width * bit_depth) + (src_x * bit_depth);
            pos_dst = ((dst_y + j + vinfo.yoffset) * finfo.line_length)
                + ((dst_x + vinfo.xoffset) * bit_depth);

            src_u64 = (uint64_t *)(src.ptr + pos_src);
            dst_u64 = (uint64_t *)(dst + pos_dst);

            for (i = 0; i < (w/2); i++) {
                value = src_u64[i];
                dst_u64[i] =  ((value>>6) & 0x00FF000000FF0000)     // Red
                    | ((value>>4) & 0x0000FF000000FF00)     // Green
                    | ((value>>2) & 0x000000FF000000FF);    // Blue
            }
        }
    } else
    */
    {
#if 0
        for(j = 0; j < h; j++) {
            pos_src = (src_y + j) * (w * 3);
            pos_dst = ((dst_y + j + vinfo.yoffset) * finfo.line_length)
                + ((dst_x + vinfo.xoffset) * bit_depth);
            for (k = 0; k < w; k++) {
                *(dst + pos_dst + 4 * k) = *(src + pos_src + 3 * k + 1);
                *(dst + pos_dst + 4 * k + 1) = *(src + pos_src + 3 * k);
                *(dst + pos_dst + 4 * k + 2) = *(src + pos_src + 3 * k + 2);
                *(dst + pos_dst + 4 * k + 3) = 0xff;
            }
        }
#else
        memcpy(dst, src, w * h * 3);
#endif

        vinfo.activate = FB_ACTIVATE_NOW;
        if (fb_index >= 3)
            fb_index = 0;
        vinfo.yoffset = 1080 * fb_index;
        vinfo.vmode &= ~FB_VMODE_YWRAP;
        ioctl(fb_fd, FBIOPAN_DISPLAY, &vinfo);
    }
    return 0;
}

int renderImageGe2d(aml_ge2d_t *amlge2d, unsigned char *src, int width, int height, uint32_t pixel_format)
{
	int ret = -1;
	int i;

	for (i=0; i<amlge2d->ge2dinfo.src_info[0].plane_number; i++) {
		memcpy(amlge2d->ge2dinfo.src_info[0].vaddr[i], src + i * width * height, amlge2d->src_size[i]);
	}

	amlge2d->ge2dinfo.src_info[0].rect.x = 0;
	amlge2d->ge2dinfo.src_info[0].rect.y = 0;
	amlge2d->ge2dinfo.src_info[0].rect.w = width;
	amlge2d->ge2dinfo.src_info[0].rect.h = height;
	amlge2d->ge2dinfo.dst_info.rect.x = 0;
	amlge2d->ge2dinfo.dst_info.rect.y = 0;
	amlge2d->ge2dinfo.dst_info.rect.w = width;
	amlge2d->ge2dinfo.dst_info.rect.h = height;
	amlge2d->ge2dinfo.dst_info.rotation = GE2D_ROTATION_0;
	amlge2d->ge2dinfo.src_info[0].layer_mode = 0;
	amlge2d->ge2dinfo.src_info[0].plane_alpha = 0xff;

	ret = aml_ge2d_process(&amlge2d->ge2dinfo);
	if (ret < 0) {
		printf("aml_ge2d_process failed!\n");
		return -1;
	}

	return 0;
}
