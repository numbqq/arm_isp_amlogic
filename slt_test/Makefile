export PATH=/home/nick/code/fenix/build/toolchains/gcc-linaro-aarch64-linux-gnu/bin/:$PATH

CC=aarch64-linux-gnu-gcc
CROSS_COMPILE=aarch64-linux-gnu-


CFLAGS=-I. -g -fPIE -Wall -pthread
ODIR=obj
OFILE=slt-isp

_OBJ = v4l2_test.o
OBJ=$(patsubst %,$(ODIR)/%,$(_OBJ))

$(ODIR)/%.o: %.c
	$(CC) -c -o $@ $< $(CFLAGS)

$(OFILE): $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) -pie

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o $(OFILE)
