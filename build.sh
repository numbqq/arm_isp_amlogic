################################################################################
#
# amlogic arm_isp drivers
#
################################################################################

# modules
D=`pwd`
STAGING_DIR=./tmp
TARGET_DIR=./tmp1
FENIX_DIR=$1
LINUX_DIR=$FENIX_DIR/linux
TARGET_KERNEL_CROSS=aarch64-linux-gnu-
KERNEL_ARCH=arm64
ARM_ISP_MODULE_DIR=kernel/amlogic/arm_isp
ARM_ISP_INSTALL_DIR=$TARGET_DIR/lib/modules/$LINUX_VERSION_PROBED/$ARM_ISP_MODULE_DIR
ARM_ISP_DEP=$TARGET_DIR/lib/modules/$LINUX_VERSION_PROBED/modules.dep
ARM_ISP_INSTALL_STAGING_DIR=$STAGING_DIR/usr/include/linux
ARM_AUTOCAP=$D/isp_module/v4l2_dev/inc/api/acamera_autocap_api.h



export PATH=$FENIX_DIR/build/toolchains/gcc-linaro-aarch64-linux-gnu/bin:$PATH

aarch64-linux-gnu-gcc -v


#define copy-arm-isp
#        $(foreach m, $(shell find $(strip $(1)) -name "*.ko"),\
#                $(shell [ ! -e $(2) ] && mkdir $(2) -p;\
#                cp $(m) $(strip $(2))/ -rfa;\
#                echo $(4)/$(notdir $(m)): >> $(3)))
#endef

V4L2_DEV_BUILD_CMDS() {
	cd $D/isp_module/v4l2_dev
	make -C $D/isp_module/v4l2_dev KDIR=$LINUX_DIR ARCH=$KERNEL_ARCH CROSS_COMPILE=$TARGET_KERNEL_CROSS
}

SENSOR_DEV_BUILD_CMDS() {	
	cd $D/isp_module/subdev/sensor
	make -C $D/isp_module/subdev/sensor KDIR=$LINUX_DIR ARCH=$KERNEL_ARCH CROSS_COMPILE=$TARGET_KERNEL_CROSS
}

IQ_DEV_BUILD_CMDS() {
	cd $D/isp_module/subdev/iq
	make-C $D/isp_module/subdev/iq KDIR=$LINUX_DIR ARCH=$KERNEL_ARCH CROSS_COMPILE=$TARGET_KERNEL_CROSS
}

LENS_DEV_BUILD_CMDS() {
	cd $D/isp_module/subdev/lens
	make -C $D/isp_module/subdev/lens KDIR=$LINUX_DIR ARCH=$KERNEL_ARCH CROSS_COMPILE=$TARGET_KERNEL_CROSS
}

ARM_ISP_BUILD_CMDS() {
	V4L2_DEV_BUILD_CMDS
	SENSOR_DEV_BUILD_CMDS
	IQ_DEV_BUILD_CMDS
	LENS_DEV_BUILD_CMDS
}

ARM_ISP_CLEAN_CMDS() {
	make -C $D/isp_module/v4l2_dev clean
	make -C $D/isp_module/subdev/sensor clean
	make -C $D/isp_module/subdev/iq clean
	make -C $D/isp_module/subdev/lens clean
}

ARM_ISP_SERVER_BIN=$D/lib/lib64/iv009_isp_64.elf

ARM_ISP_INSTALL_STAGING_CMDS() {
	if [ -f "$ARM_AUTOCAP" ]; then
		install -D -m 0644 $ARM_AUTOCAP $ARM_ISP_INSTALL_STAGING_DIR
	fi
}

ARM_ISP_INSTALL_TARGET_CMDS() {
	copy-arm-isp $D $ARM_ISP_INSTALL_DIR $ARM_ISP_DEP $ARM_ISP_MODULE_DIR

#	install -D -m 0755 $ARM_ISP_SERVER_BIN $TARGET_DIR/usr/bin/iv009_isp
#	install -D -m 0755 package/arm_isp/S30isp $TARGET_DIR/etc/init.d/
}

ARM_ISP_BUILD_CMDS
