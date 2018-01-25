# ELP_H264_UVC
Official H264 UVC driver for ELP Camera (Downloaded from ELP China)

This driver has been tested on Debian 8 on Beaglebone Black (Kernel V. linux-4.4.54-ti-r93)

Installing
- Compiling the uvc_3.3.8 first with Makefile_PC (Inheritated file eg. Makefile) with 
  > makefile
  
- Unload original uvc driver
    > sudo rmmod uvcvideo
  
- Load new UVC driver module
  > sudo insmod ./uvcvideo_h264.ko
  
 Piping out the raw H264 by H264_UVC_TestAP to stdout eg. FFMPEG 
 Modifying the H264_UVC_TestAP.c to allows this application piping out,
 
   - line 129 
      This function `static int CheckKernelVersion(void)` must bypass by force return true
      
   - line 682 adding
	    TestAp_Printf(TESTAP_DBG_USAGE, "-o, --stdout		stdout the stream\n");  
      
   - 
