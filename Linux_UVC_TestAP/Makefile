#compiler
#CC = /opt/buildroot-gcc342/bin/mipsel-linux-uclibc-gcc

#flags
#CFLAGS = -Wall -ansi -g
CFLAGS = -g -I/usr/src/linux-$(shell uname -r)/include
#CFLAGS = -g -I/usr/src/linux-2.6.36.4/include

#objects
OBJS = H264_UVC_TestAP.o h264_xu_ctrls.o v4l2uvc.o nalu.o cap_desc_parser.o cap_desc.o

#install path
INSTALL_PATH = ./

all: H264_UVC_TestAP

H264_UVC_TestAP: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $@ -lpthread

H264_UVC_TestAP.o: H264_UVC_TestAP.c h264_xu_ctrls.h
	$(CC) $(CFLAGS) -c -o $@ $<

H264_xu_ctrls.o: h264_xu_ctrls.c h264_xu_ctrls.h
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	-rm -f *.o *.ko .*.cmd .*.flags *.mod.c


