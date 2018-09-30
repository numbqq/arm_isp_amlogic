1.编译
进入slt_test目录,直接make,如果有问题,修改toolchain到你对应的toolchain
CC=/opt/gcc-linaro-aarch64-linux-gnu-4.9-2014.09_linux/bin/aarch64-linux-gnu-gcc
CROSS_COMPILE=/opt/gcc-linaro-aarch64-linux-gnu-4.9-2014.09_linux/bin/aarch64-linux-gnu-

2.运行
生成的v4l2_test,执行./v4l2_test即可运行
后面可选参数：
-y  配置pixel的允许diff,如果大于会报错(默认是3)
-N  执行到从第N帧(N,n需要同时选配,且n < N,默认21-30frame)
-n  从第n帧执行(N,n需要同时选配,且n < N,默认21-30frame)

3.结果说明：
如果出现如下log,说明frame的pixel差异大于设置的3,说明有slt fail
frame_i:delta:x temp larger than 3
frame_i表示第i帧,x表示当前的pixel差异，3就是设置的容许误差
