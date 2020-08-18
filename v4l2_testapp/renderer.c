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
		memcpy(dst, src, w * h * 3);

		vinfo.yoffset = 1080 * fb_index;
        break;
	case AFD_RENDER_MODE_LEFT_TOP_ROTATE_90:
		w = width;
		h = height;
		for(j = 0; j < h; j++) {
			pos_src = j * (w * 3);

			for (k = 0; k < w; k++) {
				pos_dst = (k * finfo.line_length);
				*(dst + pos_dst + 4 * j) = *(src + pos_src + 3 * k);
				*(dst + pos_dst + 4 * j + 1) = *(src + pos_src + 3 * k + 1);
				*(dst + pos_dst + 4 * j + 2) = *(src + pos_src + 3 * k + 2);
				*(dst + pos_dst + 4 * j + 3) = 0xff;
			}
		}
		break;

    default:
        perror("Error, invalid mode value");
        exit(1);
        break;
    }

	vinfo.activate = FB_ACTIVATE_NOW;
	if (fb_index >= 3)
		fb_index = 0;
	vinfo.vmode &= ~FB_VMODE_YWRAP;
	ioctl(fb_fd, FBIOPAN_DISPLAY, &vinfo);
    
	return 0;
}
