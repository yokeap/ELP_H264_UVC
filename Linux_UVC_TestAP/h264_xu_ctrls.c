#define N_(x) x

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>
#include "h264_xu_ctrls.h"
#include "debug.h"

extern struct H264Format *gH264fmt;

unsigned char m_CurrentFPS = 24;
unsigned char m_CurrentFPS_9422 = 30;
unsigned int  chip_id = CHIP_NONE;

#define Default_fwLen 13
const unsigned char Default_fwData[Default_fwLen] = {0x05, 0x00, 0x02, 0xD0, 0x01,		// W=1280, H=720, NumOfFrmRate=1
										   0xFF, 0xFF, 0xFF, 0xFF,			// Frame size
										   0x07, 0xA1, 0xFF, 0xFF,			// 20
											};

#define LENGTH_OF_RERVISION_XU_SYS_CTR		(7)
#define LENGTH_OF_RERVISION_XU_USR_CTR		(9)
#define RERVISION_RER9420_SERIES_CHIPID 	0x90
#define RERVISION_RER9422_SERIES_CHIPID		0x92
#define RERVISION_RER9422_DDR_64M			0x00
#define RERVISION_RER9422_DDR_16M			0x03

extern int video_get_framerate(int dev, int *framerate);
static struct uvc_xu_control_info rervision_xu_sys_ctrls[] = 
{
	{
		.entity   = UVC_GUID_RERVISION_SYS_HW_CTRL,
		.selector = XU_RERVISION_SYS_ASIC_RW,
		.index    = 0,
		.size     = 4,
		.flags    = UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_MIN | UVC_CONTROL_GET_MAX | 
		            UVC_CONTROL_GET_DEF | UVC_CONTROL_AUTO_UPDATE | UVC_CONTROL_GET_CUR
	},
	{
		.entity   = UVC_GUID_RERVISION_SYS_HW_CTRL,
		.selector = XU_RERVISION_SYS_FRAME_INFO,
		.index    = 1,
		.size     = 11,
		.flags    = UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_MIN | UVC_CONTROL_GET_MAX | 
		            UVC_CONTROL_GET_DEF | UVC_CONTROL_AUTO_UPDATE | UVC_CONTROL_GET_CUR
	},
	{
		.entity   = UVC_GUID_RERVISION_SYS_HW_CTRL,
		.selector = XU_RERVISION_SYS_H264_CTRL,
		.index    = 2,
		.size     = 11,
		.flags    = UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_MIN | UVC_CONTROL_GET_MAX | 
		            UVC_CONTROL_GET_DEF | UVC_CONTROL_AUTO_UPDATE | UVC_CONTROL_GET_CUR
	},
	{
		.entity   = UVC_GUID_RERVISION_SYS_HW_CTRL,
		.selector = XU_RERVISION_SYS_MJPG_CTRL,
		.index    = 3,
		.size     = 11,
		.flags    = UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_CUR
	},
	{
		.entity   = UVC_GUID_RERVISION_SYS_HW_CTRL,
		.selector = XU_RERVISION_SYS_OSD_CTRL,
		.index    = 4,
		.size     = 11,
		.flags    = UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_CUR
	},
	{
		.entity   = UVC_GUID_RERVISION_SYS_HW_CTRL,
		.selector = XU_RERVISION_SYS_OSD_CTRL,
		.index    = 5,
		.size     = 11,
		.flags    = UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_MIN | UVC_CONTROL_GET_MAX | 
		            UVC_CONTROL_GET_DEF | UVC_CONTROL_AUTO_UPDATE | UVC_CONTROL_GET_CUR
	},
	{
		.entity   = UVC_GUID_RERVISION_SYS_HW_CTRL,
		.selector = XU_RERVISION_SYS_MOTION_DETECTION,
		.index    = 6,
		.size     = 11,
		.flags    = UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_CUR
	},
	{
		.entity   = UVC_GUID_RERVISION_SYS_HW_CTRL,
		.selector = XU_RERVISION_SYS_IMG_SETTING,
		.index    = 7,
		.size     = 11,
		.flags    = UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_CUR
	},
};

static struct uvc_xu_control_info rervision_xu_usr_ctrls[] =
{
    {
      .entity   = UVC_GUID_RERVISION_USR_HW_CTRL,
      .selector = XU_RERVISION_USR_FRAME_INFO,
      .index    = 0,
      .size     = 11,
      .flags    = UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_MIN | UVC_CONTROL_GET_MAX |
                  UVC_CONTROL_GET_DEF | UVC_CONTROL_AUTO_UPDATE | UVC_CONTROL_GET_CUR
    },
    {
      .entity   = UVC_GUID_RERVISION_USR_HW_CTRL,
      .selector = XU_RERVISION_USR_H264_CTRL,
      .index    = 1,
      .size     = 11,
      .flags    = UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_CUR
    },
    {
      .entity   = UVC_GUID_RERVISION_USR_HW_CTRL,
      .selector = XU_RERVISION_USR_MJPG_CTRL,
      .index    = 2,
      .size     = 11,
      .flags    = UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_CUR
    },
    {
      .entity   = UVC_GUID_RERVISION_USR_HW_CTRL,
      .selector = XU_RERVISION_USR_OSD_CTRL,
      .index    = 3,
      .size     = 11,
      .flags    = UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_MIN | UVC_CONTROL_GET_MAX |
                  UVC_CONTROL_GET_DEF | UVC_CONTROL_AUTO_UPDATE | UVC_CONTROL_GET_CUR
    },
    {
      .entity   = UVC_GUID_RERVISION_USR_HW_CTRL,
      .selector = XU_RERVISION_USR_MOTION_DETECTION,
      .index    = 4,
      .size     = 24,
      .flags    = UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_CUR
    },
    {
      .entity   = UVC_GUID_RERVISION_USR_HW_CTRL,
      .selector = XU_RERVISION_USR_IMG_SETTING,
      .index    = 5,
      .size     = 11,
      .flags    = UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_CUR
    },
    {
      .entity   = UVC_GUID_RERVISION_USR_HW_CTRL,
      .selector = XU_RERVISION_USR_MULTI_STREAM_CTRL,
      .index    = 6,
      .size     = 11,
      .flags    = UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_CUR
    },	
    {
      .entity   = UVC_GUID_RERVISION_USR_HW_CTRL,
      .selector = XU_RERVISION_USR_GPIO_CTRL,
      .index    = 7,
      .size     = 11,
      .flags    = UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_CUR
    },	
    {
      .entity   = UVC_GUID_RERVISION_USR_HW_CTRL,
      .selector = XU_RERVISION_USR_DYNAMIC_FPS_CTRL,
      .index    = 8,
      .size     = 11,
      .flags    = UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_CUR
    },	
};

//  RERVISION XU system Ctrls Mapping
static struct uvc_xu_control_mapping rervision_xu_sys_mappings[] = 
{
	{
		.id        = V4L2_CID_ASIC_RW_RERVISION,
		.name      = "RERVISION: Asic Read",
		.entity    = UVC_GUID_RERVISION_SYS_HW_CTRL,
		.selector  = XU_RERVISION_SYS_ASIC_RW,
		.size      = 4,
		.offset    = 0,
		.v4l2_type = V4L2_CTRL_TYPE_INTEGER,
		.data_type = UVC_CTRL_DATA_TYPE_SIGNED
	},
	{
		.id        = V4L2_CID_FLASH_CTRL,
		.name      = "RERVISION: Flash Control",
		.entity    = UVC_GUID_RERVISION_SYS_HW_CTRL,
		.selector  = XU_RERVISION_SYS_FLASH_CTRL,
		.size      = 11,
		.offset    = 0,
		.v4l2_type = V4L2_CTRL_TYPE_INTEGER,
		.data_type = UVC_CTRL_DATA_TYPE_SIGNED
	},
	{
		.id        = V4L2_CID_FRAME_INFO_RERVISION,
		.name      = "RERVISION: H264 Format",
		.entity    = UVC_GUID_RERVISION_SYS_HW_CTRL,
		.selector  = XU_RERVISION_SYS_FRAME_INFO,
		.size      = 11,
		.offset    = 0,
		.v4l2_type = V4L2_CTRL_TYPE_INTEGER,
		.data_type = UVC_CTRL_DATA_TYPE_RAW
	},
	{
		.id        = V4L2_CID_H264_CTRL_RERVISION,
		.name      = "RERVISION: H264 Control",
		.entity    = UVC_GUID_RERVISION_SYS_HW_CTRL,
		.selector  = XU_RERVISION_SYS_H264_CTRL,
		.size      = 11,
		.offset    = 0,
		.v4l2_type = V4L2_CTRL_TYPE_INTEGER,
		.data_type = UVC_CTRL_DATA_TYPE_UNSIGNED
	},
	{
		.id        = V4L2_CID_MJPG_CTRL_RERVISION,
		.name      = "RERVISION: MJPG Control",
		.entity    = UVC_GUID_RERVISION_SYS_HW_CTRL,
		.selector  = XU_RERVISION_SYS_MJPG_CTRL,
		.size      = 11,
		.offset    = 0,
		.v4l2_type = V4L2_CTRL_TYPE_INTEGER,
		.data_type = UVC_CTRL_DATA_TYPE_UNSIGNED
	},
	{
		.id        = V4L2_CID_OSD_CTRL_RERVISION,
		.name      = "RERVISION: OSD Control",
		.entity    = UVC_GUID_RERVISION_SYS_HW_CTRL,
		.selector  = XU_RERVISION_SYS_OSD_CTRL,
		.size      = 11,
		.offset    = 0,
		.v4l2_type = V4L2_CTRL_TYPE_INTEGER,
		.data_type = UVC_CTRL_DATA_TYPE_RAW
	},
	{
		.id        = V4L2_CID_MOTION_DETECTION_RERVISION,
		.name      = "RERVISION: Motion Detection",
		.entity    = UVC_GUID_RERVISION_SYS_HW_CTRL,
		.selector  = XU_RERVISION_SYS_MOTION_DETECTION,
		.size      = 11,
		.offset    = 0,
		.v4l2_type = V4L2_CTRL_TYPE_INTEGER,
		.data_type = UVC_CTRL_DATA_TYPE_UNSIGNED
	},
	{
		.id        = V4L2_CID_IMG_SETTING_RERVISION,
		.name      = "RERVISION: Image Setting",
		.entity    = UVC_GUID_RERVISION_SYS_HW_CTRL,
		.selector  = XU_RERVISION_SYS_IMG_SETTING,
		.size      = 11,
		.offset    = 0,
		.v4l2_type = V4L2_CTRL_TYPE_INTEGER,
		.data_type = UVC_CTRL_DATA_TYPE_UNSIGNED
	},
};

// RERVISION XU user Ctrls Mapping
static struct uvc_xu_control_mapping rervision_xu_usr_mappings[] =
{
    {
      .id        = V4L2_CID_FRAME_INFO_RERVISION,
      .name      = "RERVISION: H264 Format",
      .entity    = UVC_GUID_RERVISION_USR_HW_CTRL,
      .selector  = XU_RERVISION_USR_FRAME_INFO,
      .size      = 11,
      .offset    = 0,
      .v4l2_type = V4L2_CTRL_TYPE_INTEGER,
      .data_type = UVC_CTRL_DATA_TYPE_RAW
    },
    {
      .id        = V4L2_CID_H264_CTRL_RERVISION,
      .name      = "RERVISION: H264 Control",
      .entity    = UVC_GUID_RERVISION_USR_HW_CTRL,
      .selector  = XU_RERVISION_USR_H264_CTRL,
      .size      = 11,
      .offset    = 0,
      .v4l2_type = V4L2_CTRL_TYPE_INTEGER,
      .data_type = UVC_CTRL_DATA_TYPE_UNSIGNED
    },
    {
      .id        = V4L2_CID_MJPG_CTRL_RERVISION,
      .name      = "RERVISION: MJPG Control",
      .entity    = UVC_GUID_RERVISION_USR_HW_CTRL,
      .selector  = XU_RERVISION_USR_MJPG_CTRL,
      .size      = 11,
      .offset    = 0,
      .v4l2_type = V4L2_CTRL_TYPE_INTEGER,
      .data_type = UVC_CTRL_DATA_TYPE_UNSIGNED
    },
    {
      .id        = V4L2_CID_OSD_CTRL_RERVISION,
      .name      = "RERVISION: OSD Control",
      .entity    = UVC_GUID_RERVISION_USR_HW_CTRL,
      .selector  = XU_RERVISION_USR_OSD_CTRL,
      .size      = 11,
      .offset    = 0,
      .v4l2_type = V4L2_CTRL_TYPE_INTEGER,
      .data_type = UVC_CTRL_DATA_TYPE_RAW
    },
    {
      .id        = V4L2_CID_MOTION_DETECTION_RERVISION,
      .name      = "RERVISION: Motion Detection",
      .entity    = UVC_GUID_RERVISION_USR_HW_CTRL,
      .selector  = XU_RERVISION_USR_MOTION_DETECTION,
      .size      = 24,
      .offset    = 0,
      .v4l2_type = V4L2_CTRL_TYPE_INTEGER,
      .data_type = UVC_CTRL_DATA_TYPE_UNSIGNED
    },
    {
      .id        = V4L2_CID_IMG_SETTING_RERVISION,
      .name      = "RERVISION: Image Setting",
      .entity    = UVC_GUID_RERVISION_USR_HW_CTRL,
      .selector  = XU_RERVISION_USR_IMG_SETTING,
      .size      = 11,
      .offset    = 0,
      .v4l2_type = V4L2_CTRL_TYPE_INTEGER,
      .data_type = UVC_CTRL_DATA_TYPE_UNSIGNED
    },
    {
      .id        = V4L2_CID_MULTI_STREAM_CTRL_RERVISION,
      .name      = "RERVISION: Multi Stram Control ",
      .entity    = UVC_GUID_RERVISION_USR_HW_CTRL,
      .selector  = XU_RERVISION_USR_MULTI_STREAM_CTRL,
      .size      = 11,
      .offset    = 0,
      .v4l2_type = V4L2_CTRL_TYPE_INTEGER,
      .data_type = UVC_CTRL_DATA_TYPE_UNSIGNED
    },	
    {
      .id        = V4L2_CID_GPIO_CTRL_RERVISION,
      .name      = "RERVISION: GPIO Control ",
      .entity    = UVC_GUID_RERVISION_USR_HW_CTRL,
      .selector  = XU_RERVISION_USR_GPIO_CTRL,
      .size      = 11,
      .offset    = 0,
      .v4l2_type = V4L2_CTRL_TYPE_INTEGER,
      .data_type = UVC_CTRL_DATA_TYPE_UNSIGNED
    },	
    {
      .id        = V4L2_CID_DYNAMIC_FPS_CTRL_RERVISION,
      .name      = "RERVISION: Dynamic fps Control ",
      .entity    = UVC_GUID_RERVISION_USR_HW_CTRL,
      .selector  = XU_RERVISION_USR_DYNAMIC_FPS_CTRL,
      .size      = 11,
      .offset    = 0,
      .v4l2_type = V4L2_CTRL_TYPE_INTEGER,
      .data_type = UVC_CTRL_DATA_TYPE_UNSIGNED
    },	
};

int XU_Set_Cur(int fd, __u8 xu_unit, __u8 xu_selector, __u16 xu_size, __u8 *xu_data)
{
	int err=0;
#if LINUX_VERSION_CODE > KERNEL_VERSION (3, 0, 36)
	struct uvc_xu_control_query xctrl;
	xctrl.unit = xu_unit;
	xctrl.selector = xu_selector;
	xctrl.query = UVC_SET_CUR;
	xctrl.size = xu_size;
	xctrl.data = xu_data;
	err=ioctl(fd, UVCIOC_CTRL_QUERY, &xctrl);
#else
	struct uvc_xu_control xctrl;	
	xctrl.unit = xu_unit;
	xctrl.selector = xu_selector;
	xctrl.size = xu_size;
	xctrl.data = xu_data;
	err=ioctl(fd, UVCIOC_CTRL_SET, &xctrl);
#endif		
	return err;
}

int XU_Get_Cur(int fd, __u8 xu_unit, __u8 xu_selector, __u16 xu_size, __u8 *xu_data)
{
	int err=0;
#if LINUX_VERSION_CODE > KERNEL_VERSION (3, 0, 36)
	struct uvc_xu_control_query xctrl;
	xctrl.unit = xu_unit;
	xctrl.selector = xu_selector;
	xctrl.query = UVC_GET_CUR;
	xctrl.size = xu_size;
	xctrl.data = xu_data;
	err=ioctl(fd, UVCIOC_CTRL_QUERY, &xctrl);
#else
	struct uvc_xu_control xctrl;	
	xctrl.unit = xu_unit;
	xctrl.selector = xu_selector;
	xctrl.size = xu_size;
	xctrl.data = xu_data;
	err=ioctl(fd, UVCIOC_CTRL_GET, &xctrl);
#endif	
	return err;
}


// XU ctrls ----------------------------------------------------------

int XU_Ctrl_Add(int fd, struct uvc_xu_control_info *info, struct uvc_xu_control_mapping *map) 
{
	int i=0;
	int err=0;
	
	/* try to add controls listed */
#if LINUX_VERSION_CODE <= KERNEL_VERSION (3, 0, 36)
	TestAp_Printf(TESTAP_DBG_FLOW, "Adding XU Ctrls - %s\n", map->name);
	if ((err=ioctl(fd, UVCIOC_CTRL_ADD, info)) < 0 ) 
	{
		if (errno == EEXIST ) 
		{	
			TestAp_Printf(TESTAP_DBG_ERR, "UVCIOC_CTRL_ADD - Ignored, uvc driver had already defined\n");
			return (-EEXIST);
		}
		else if (errno == EACCES)
		{
			TestAp_Printf(TESTAP_DBG_ERR, "Need admin previledges for adding extension unit(XU) controls\n");
			TestAp_Printf(TESTAP_DBG_ERR, "please run 'RERVISION_UVC_TestAP --add_ctrls' as root (or with sudo)\n");
			return  (-1);
		}
		else perror("Control exists");
	}
#endif	
	/* after adding the controls, add the mapping now */
	TestAp_Printf(TESTAP_DBG_FLOW, "Mapping XU Ctrls - %s\n", map->name);
	if ((err=ioctl(fd, UVCIOC_CTRL_MAP, map)) < 0) 
	{
		if ((errno!=EEXIST) && (errno != EACCES))
		{
			TestAp_Printf(TESTAP_DBG_ERR, "UVCIOC_CTRL_MAP - Error(err= %d)\n", err);
			return (-2);
		}
		else if (errno == EACCES)
		{
			TestAp_Printf(TESTAP_DBG_ERR, "Need admin previledges for adding extension unit(XU) controls\n");
			TestAp_Printf(TESTAP_DBG_ERR, "please run 'RERVISION_UVC_TestAP --add_ctrls' as root (or with sudo)\n");
			return  (-1);
		}
		else perror("Mapping exists");
	}
	
	return 0;
}

int XU_Init_Ctrl(int fd) 
{
	int i=0;
	int err=0;
	int length;
	struct uvc_xu_control_info *xu_infos;
	struct uvc_xu_control_mapping *xu_mappings;
	
	// Add xu READ ASIC first
	err = XU_Ctrl_Add(fd, &rervision_xu_sys_ctrls[i], &rervision_xu_sys_mappings[i]);
	if (err == EEXIST){}
	else if (err < 0) {return err;}

	// Read chip ID
	err = XU_Ctrl_ReadChipID(fd);
	if (err < 0) return err;


	// Add xu flash control
	i++;
	err = XU_Ctrl_Add(fd, &rervision_xu_sys_ctrls[i], &rervision_xu_sys_mappings[i]);
	if (err == EEXIST){}
	else if (err < 0) {return err;}


	// Decide which xu set had been add
	if(chip_id == CHIP_RER9420)
	{
		xu_infos = rervision_xu_sys_ctrls;
		xu_mappings = rervision_xu_sys_mappings;
		i = 2;
		length = LENGTH_OF_RERVISION_XU_SYS_CTR;
		TestAp_Printf(TESTAP_DBG_FLOW, "RER9420\n");
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		xu_infos = rervision_xu_usr_ctrls;
		xu_mappings = rervision_xu_usr_mappings;
		i = 0;
		length = LENGTH_OF_RERVISION_XU_USR_CTR;
		TestAp_Printf(TESTAP_DBG_FLOW, "RER9422\n");
	}
	else
	{
		TestAp_Printf(TESTAP_DBG_ERR, "Unknown chip id 0x%x\n", chip_id);
		return -1;
	}
	
	// Add other xu accroding chip ID
	for ( ; i<length; i++ ) 
	{
		err = XU_Ctrl_Add(fd, &xu_infos[i], &xu_mappings[i]);
		//if (err < 0) break;
	} 
	return 0;
}

int XU_Ctrl_ReadChipID(int fd)
{
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Ctrl_ReadChipID ==>\n");
	int ret = 0;
	int err = 0;
	__u8 ctrldata[4];

	//uvc_xu_control parmeters
	__u8 xu_unit= 3; 
	__u8 xu_selector= XU_RERVISION_SYS_ASIC_RW;
	__u16 xu_size= 4;
	__u8 *xu_data= ctrldata;


	xu_data[0] = 0x1f;
	xu_data[1] = 0x10;
	xu_data[2] = 0x0;
	xu_data[3] = 0xFF;		/* Dummy Write */
	
	/* Dummy Write */
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR, "  ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		return err;
	}

	/* Asic Read */
	xu_data[3] = 0x00;
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR, "   ioctl(UVCIOC_CTRL_GET) FAILED (%i)  \n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR, "    Invalid arguments\n");
		return err;
	}
	
	TestAp_Printf(TESTAP_DBG_FLOW, "   == XU_Ctrl_ReadChipID Success == \n");
	TestAp_Printf(TESTAP_DBG_FLOW, "      ASIC READ data[0] : %x\n", xu_data[0]);
	TestAp_Printf(TESTAP_DBG_FLOW, "      ASIC READ data[1] : %x\n", xu_data[1]);
	TestAp_Printf(TESTAP_DBG_FLOW, "      ASIC READ data[2] : %x (Chip ID)\n", xu_data[2]);
	TestAp_Printf(TESTAP_DBG_FLOW, "      ASIC READ data[3] : %x\n", xu_data[3]);
	
	if(xu_data[2] == RERVISION_RER9420_SERIES_CHIPID)
	{
		chip_id = CHIP_RER9420;
	}	
	if(xu_data[2] == RERVISION_RER9422_SERIES_CHIPID)
	{
		xu_data[0] = 0x07;		//DRAM SIZE
		xu_data[1] = 0x16;
		xu_data[2] = 0x0;
		xu_data[3] = 0xFF;		/* Dummy Write */

		/* Dummy Write */
		if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
		{
			TestAp_Printf(TESTAP_DBG_ERR, "  ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
			return err;
		}

		/* Asic Read */
		xu_data[3] = 0x00;
		if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
		{
			TestAp_Printf(TESTAP_DBG_ERR, "   ioctl(UVCIOC_CTRL_GET) FAILED (%i)  \n",err);
			if(err==EINVAL)
				TestAp_Printf(TESTAP_DBG_ERR, "    Invalid arguments\n");
			return err;
		}
		
		if(xu_data[2] == RERVISION_RER9422_DDR_64M)
			chip_id = CHIP_RER9422;
		else if(xu_data[2] == RERVISION_RER9422_DDR_16M)
			chip_id = CHIP_RER9421;			
	}
	
	TestAp_Printf(TESTAP_DBG_FLOW, "ChipID = %d\n",chip_id);
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Ctrl_ReadChipID <==\n");
	return ret;
}

int H264_GetFormat(int fd)
{
	TestAp_Printf(TESTAP_DBG_FLOW, "H264_GetFormat ==>\n");
	int i,j;
	int iH264FormatCount = 0;
	int success = 1;
	
	unsigned char *fwData = NULL;
	unsigned short fwLen = 0;

	// Init H264 XU Ctrl Format
	if( XU_H264_InitFormat(fd) < 0 )
	{
		TestAp_Printf(TESTAP_DBG_ERR, " H264 XU Ctrl Format failed , use default Format\n");
		fwLen = Default_fwLen;
		fwData = (unsigned char *)calloc(fwLen,sizeof(unsigned char));
		memcpy(fwData, Default_fwData, fwLen);
		goto Skip_XU_GetFormat;
	}

	// Probe : Get format through XU ctrl
	success = XU_H264_GetFormatLength(fd, &fwLen);
	if( success < 0 || fwLen <= 0)
	{
		TestAp_Printf(TESTAP_DBG_ERR, " XU Get Format Length failed !\n");
	}
	TestAp_Printf(TESTAP_DBG_FLOW, "fwLen = 0x%x\n", fwLen);
	// alloc memory
	fwData = (unsigned char *)calloc(fwLen,sizeof(unsigned char));

	if( XU_H264_GetFormatData(fd, fwData,fwLen) < 0 )
	{
		TestAp_Printf(TESTAP_DBG_ERR, " XU Get Format Data failed !\n");
	}

Skip_XU_GetFormat :
	// Get H.264 format count
	iH264FormatCount = H264_CountFormat(fwData, fwLen);

	TestAp_Printf(TESTAP_DBG_FLOW, "H264_GetFormat ==> FormatCount : %d \n", iH264FormatCount);

	if(iH264FormatCount>0)
		gH264fmt = (struct H264Format *)malloc(sizeof(struct H264Format)*iH264FormatCount);
	else
	{
		TestAp_Printf(TESTAP_DBG_ERR,"Get Resolution Data Failed\n");
	}

	// Parse & Save Size/Framerate into structure
	success = H264_ParseFormat(fwData, fwLen, gH264fmt);

	if(success)
	{
		for(i=0; i<iH264FormatCount; i++)
		{
			TestAp_Printf(TESTAP_DBG_FLOW, "Format index: %d --- (%d x %d) ---\n", i+1, gH264fmt[i].wWidth, gH264fmt[i].wHeight);
			for(j=0; j<gH264fmt[i].fpsCnt; j++)
			{
				if(chip_id == CHIP_RER9420)
				{
					TestAp_Printf(TESTAP_DBG_FLOW, "(%d) %2d fps\n", j+1, H264_GetFPS(gH264fmt[i].FrPay[j]));
				}
				else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
				{
					TestAp_Printf(TESTAP_DBG_FLOW, "(%d) %2d fps\n", j+1, H264_GetFPS(gH264fmt[i].FrPay[j*2]));
				}
			}
		}
	}

	if(fwData)
	{
		free(fwData);
		fwData = NULL;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "H264_GetFormat <== \n");

	return success;
}

int H264_CountFormat(unsigned char *Data, int len)
{
	int fmtCnt = 0;
	int fpsCnt = 0;
	int cur_len = 0;
	int cur_fmtid = 0;
	int cur_fpsNum = 0;
	
	if( Data == NULL || len == 0)
		return 0;

	// count Format numbers
	while(cur_len < len)
	{
		cur_fpsNum = Data[cur_len+4];
		
		TestAp_Printf(TESTAP_DBG_FLOW, "H264_CountFormat ==> cur_len = %d, cur_fpsNum= %d\n", cur_len , cur_fpsNum);

		if(chip_id == CHIP_RER9420)
		{
			cur_len += 9 + cur_fpsNum * 4;
		}
		else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
		{
			cur_len += 9 + cur_fpsNum * 6;
		}

		fmtCnt++;
	}
	
	if(cur_len != len)
	{
		TestAp_Printf(TESTAP_DBG_FLOW, "H264_CountFormat ==> cur_len = %d, fwLen= %d\n", cur_len , len);
		return 0;
	}
	
	TestAp_Printf(TESTAP_DBG_FLOW, "  ========  fmtCnt=%d   ======== \n",fmtCnt);
	return fmtCnt;
}
int H264_ParseFormat(unsigned char *Data, int len, struct H264Format *fmt)
{
	TestAp_Printf(TESTAP_DBG_FLOW, "H264_ParseFormat ==>\n");
	int fpsCnt = 0;
	int cur_len = 0;
	int cur_fmtid = 0;
	int cur_fpsNum = 0;
	int i;

	while(cur_len < len)
	{
		// Copy Size
		fmt[cur_fmtid].wWidth  = ((unsigned short)Data[cur_len]<<8)   + (unsigned short)Data[cur_len+1];
		fmt[cur_fmtid].wHeight = ((unsigned short)Data[cur_len+2]<<8) + (unsigned short)Data[cur_len+3];
		fmt[cur_fmtid].fpsCnt  = Data[cur_len+4];
		fmt[cur_fmtid].FrameSize =	((unsigned int)Data[cur_len+5] << 24) | 
						((unsigned int)Data[cur_len+6] << 16) | 
						((unsigned int)Data[cur_len+7] << 8 ) | 
						((unsigned int)Data[cur_len+8]);

		TestAp_Printf(TESTAP_DBG_FLOW, "Data[5~8]: 0x%02x%02x%02x%02x \n", Data[cur_len+5],Data[cur_len+6],Data[cur_len+7],Data[cur_len+8]);
		TestAp_Printf(TESTAP_DBG_FLOW, "fmt[%d].FrameSize: 0x%08x \n", cur_fmtid, fmt[cur_fmtid].FrameSize);

		// Alloc memory for Frame rate 
		cur_fpsNum = Data[cur_len+4];
		
		if(chip_id == CHIP_RER9420)
		{
			fmt[cur_fmtid].FrPay = (unsigned int *)malloc(sizeof(unsigned int)*cur_fpsNum);
		}
		else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
		{
			fmt[cur_fmtid].FrPay = (unsigned int *)malloc(sizeof(unsigned int)*cur_fpsNum*2);
		}
		
		for(i=0; i<cur_fpsNum; i++)
		{
			if(chip_id == CHIP_RER9420)
			{
				fmt[cur_fmtid].FrPay[i] =	(unsigned int)Data[cur_len+9+i*4]   << 24 | 
								(unsigned int)Data[cur_len+9+i*4+1] << 16 |
								(unsigned int)Data[cur_len+9+i*4+2] << 8  |
								(unsigned int)Data[cur_len+9+i*4+3] ;
			
			//TestAp_Printf(TESTAP_DBG_FLOW, "fmt[cur_fmtid].FrPay[%d]: 0x%08x \n", i, fmt[cur_fmtid].FrPay[i]);
			}
			else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
			{
				fmt[cur_fmtid].FrPay[i*2] =	(unsigned int)Data[cur_len+9+i*6]   << 8 | (unsigned int)Data[cur_len+9+i*6+1];
				fmt[cur_fmtid].FrPay[i*2+1] =	(unsigned int)Data[cur_len+9+i*6+2]   << 24 | 
								(unsigned int)Data[cur_len+9+i*6+3] << 16 |
								(unsigned int)Data[cur_len+9+i*6+4] << 8  |
								(unsigned int)Data[cur_len+9+i*6+5] ;

				TestAp_Printf(TESTAP_DBG_FLOW, "fmt[cur_fmtid].FrPay[%d]: 0x%04x  0x%08x \n", i, fmt[cur_fmtid].FrPay[i*2], fmt[cur_fmtid].FrPay[i*2+1]);
			}
		}
		
		// Do next format
		if(chip_id == CHIP_RER9420)
		{
			cur_len += 9 + cur_fpsNum * 4;
		}
		else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
		{
			cur_len += 9 + cur_fpsNum * 6;
		}
		cur_fmtid++;
	}
	if(cur_len != len)
	{
		TestAp_Printf(TESTAP_DBG_ERR,"H264_ParseFormat <==  fail \n");
		return 0;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "H264_ParseFormat <==\n");
	return 1;
}

int H264_GetFPS(unsigned int FrPay)
{
	int fps = 0;

	if(chip_id == CHIP_RER9420)
	{
		//TestAp_Printf(TESTAP_DBG_FLOW, "H264_GetFPS==> FrPay = 0x%04x\n", (FrPay & 0xFFFF0000)>>16);

		unsigned short frH = (FrPay & 0xFF000000)>>16;
		unsigned short frL = (FrPay & 0x00FF0000)>>16;
		unsigned short fr = (frH|frL);
	
		//TestAp_Printf(TESTAP_DBG_FLOW, "FrPay: 0x%x -> fr = 0x%x\n",FrPay,fr);
	
		fps = ((unsigned int)10000000/fr)>>8;

		//TestAp_Printf(TESTAP_DBG_FLOW, "fps : %d\n", fps);
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		//TestAp_Printf(TESTAP_DBG_FLOW, "H264_GetFPS==> Fr = 0x%04x\n", (unsigned short)FrPay);

		fps = ((unsigned int)10000000/(unsigned short)FrPay)>>8;

		//TestAp_Printf(TESTAP_DBG_FLOW, "fps : %d\n", fps);
	}

	return fps;
}


int XU_H264_InitFormat(int fd)
{
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_InitFormat ==>\n");
	int i = 0;
	int ret = 0;
	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= 0; 
	__u8 xu_selector= 0;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	if(chip_id == CHIP_RER9420)
	{
		xu_unit = XU_RERVISION_SYS_ID;
		xu_selector = XU_RERVISION_SYS_FRAME_INFO;
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		xu_unit = XU_RERVISION_USR_ID;
		xu_selector = XU_RERVISION_USR_FRAME_INFO;
	}

	// Switch command : FMT_NUM_INFO
	// data[0] = 0x01;
	xu_data[0] = 0x9A;
	xu_data[1] = 0x01;
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"   Set Switch command : FMT_NUM_INFO FAILED (%i)\n",err);
		return err;
	}

	//xu_data[0] = 0;
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0)
	{	
		TestAp_Printf(TESTAP_DBG_ERR,"   ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		return err;
	}

	for(i=0; i<xu_size; i++)
		TestAp_Printf(TESTAP_DBG_FLOW, " Get Data[%d] = 0x%x\n",i, xu_data[i]);
	TestAp_Printf(TESTAP_DBG_FLOW, " ubH264Idx_S1 = %d\n", xu_data[5]);

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_InitFormat <== Success\n");
	return ret;
}

int XU_H264_GetFormatLength(int fd, unsigned short *fwLen)
{
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_GetFormatLength ==>\n");
	int i = 0;
	int ret = 0;
	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= 0; 
	__u8 xu_selector= 0;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	if(chip_id == CHIP_RER9420)
	{
		xu_unit = XU_RERVISION_SYS_ID;
		xu_selector = XU_RERVISION_SYS_FRAME_INFO;
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		xu_unit = XU_RERVISION_USR_ID;
		xu_selector = XU_RERVISION_USR_FRAME_INFO;
	}	
		
	// Switch command : FMT_Data Length_INFO
	xu_data[0] = 0x9A;
	xu_data[1] = 0x02;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) >= 0) 
	{
		//for(i=0; i<11; i++)	xu_data[i] = 0;		// clean data
		memset(xu_data, 0, xu_size);

		if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) >= 0)
		{
			for (i=0 ; i<11 ; i+=2)
				TestAp_Printf(TESTAP_DBG_FLOW, " Get Data[%d] = 0x%x\n", i, (xu_data[i]<<8)+xu_data[i+1]);

			// Get H.264 format length
			*fwLen = ((unsigned short)xu_data[6]<<8) + xu_data[7];
			TestAp_Printf(TESTAP_DBG_FLOW, " H.264 format Length = 0x%x\n", *fwLen);
		}
		else
		{
			TestAp_Printf(TESTAP_DBG_ERR,"   ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
			return err;
		}
	}
	else
	{
		TestAp_Printf(TESTAP_DBG_ERR,"   Set Switch command : FMT_Data Length_INFO FAILED (%i)\n",err);
		return err;
	}
	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_GetFormatLength <== Success\n");
	return ret;
}

int XU_H264_GetFormatData(int fd, unsigned char *fwData, unsigned short fwLen)
{
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_GetFormatData ==>\n");
	int i = 0;
	int ret = 0;
	int err = 0;
	__u8 ctrldata[11]={0};

	int loop = 0;
	int LoopCnt  = (fwLen%11) ? (fwLen/11)+1 : (fwLen/11) ;
	int Copyleft = fwLen;

	//uvc_xu_control parmeters
	__u8 xu_unit= 0; 
	__u8 xu_selector= 0;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	if(chip_id == CHIP_RER9420)
	{
		xu_unit = XU_RERVISION_SYS_ID;
		xu_selector = XU_RERVISION_SYS_FRAME_INFO;
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		xu_unit = XU_RERVISION_USR_ID;
		xu_selector = XU_RERVISION_USR_FRAME_INFO;
	}

	// Switch command
	xu_data[0] = 0x9A;		
	xu_data[1] = 0x03;		// FRM_DATA_INFO
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0)
	{
		TestAp_Printf(TESTAP_DBG_ERR,"   Set Switch command : FRM_DATA_INFO FAILED (%i)\n",err);
		return err;
	}

	// Get H.264 Format Data
	xu_data[0] = 0x02;		// Stream: 1
	xu_data[1] = 0x01;		// Format: 1 (H.264)

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) >= 0) 
	{
		// Read all H.264 format data
		for(loop = 0 ; loop < LoopCnt ; loop ++)
		{
			for(i=0; i<11; i++)	xu_data[i] = 0;		// clean data
			
			TestAp_Printf(TESTAP_DBG_FLOW, "--> Loop : %d <--\n",  loop);
			
			if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) >= 0)
			{
				for (i=0 ; i<11 ; i++)
					TestAp_Printf(TESTAP_DBG_FLOW, " Data[%d] = 0x%x\n", i, xu_data[i]);

				// Copy Data
				if(Copyleft >= 11)
				{
					memcpy( &fwData[loop*11] , xu_data, 11);
					Copyleft -= 11;
				}
				else
				{
					memcpy( &fwData[loop*11] , xu_data, Copyleft);
					Copyleft = 0;
				}
			}
			else
			{
				TestAp_Printf(TESTAP_DBG_ERR,"   ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
				return err;
			}
		}
	}
	else
	{
		TestAp_Printf(TESTAP_DBG_ERR,"   Set Switch command : FRM_DATA_INFO FAILED (%i)\n",err);
		return err;
	}
	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_GetFormatData <== Success\n");
	return ret;
}

int XU_H264_SetFormat(int fd, struct Cur_H264Format fmt)
{
	// Need to get H264 format first
	if(gH264fmt==NULL)
	{
		TestAp_Printf(TESTAP_DBG_FLOW, "RERVISION_UVC_TestAP @XU_H264_SetFormat : Do XU_H264_GetFormat before setting H264 format\n");
		return -EINVAL;
	}

	if(chip_id == CHIP_RER9420)
	{
		TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_SetFormat ==> %d-%d => (%d x %d):%d fps\n", 
				fmt.FmtId+1, fmt.FrameRateId+1, 
				gH264fmt[fmt.FmtId].wWidth, 
				gH264fmt[fmt.FmtId].wHeight, 
				H264_GetFPS(gH264fmt[fmt.FmtId].FrPay[fmt.FrameRateId]));
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_SetFormat ==> %d-%d => (%d x %d):%d fps\n", 
				fmt.FmtId+1, fmt.FrameRateId+1, 
				gH264fmt[fmt.FmtId].wWidth, 
				gH264fmt[fmt.FmtId].wHeight, 
				H264_GetFPS(gH264fmt[fmt.FmtId].FrPay[fmt.FrameRateId*2]));
	}

	int ret = 0;
	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= 0; 
	__u8 xu_selector= 0;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	if(chip_id == CHIP_RER9420)
	{
		xu_unit = XU_RERVISION_SYS_ID;
		xu_selector = XU_RERVISION_SYS_FRAME_INFO;
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		xu_unit = XU_RERVISION_USR_ID;
		xu_selector = XU_RERVISION_USR_FRAME_INFO;
	}
		
	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x21;				// Commit_INFO
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Set_FMT ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set H.264 format setting data
	xu_data[0] = 0x02;				// Stream : 1
	xu_data[1] = 0x01;				// Format index : 1 (H.264) 
	xu_data[2] = fmt.FmtId + 1;		// Frame index (Resolution index), firmware index starts from 1
//	xu_data[3] = ( gH264fmt[fmt.FmtId].FrPay[fmt.FrameRateId] & 0xFF000000 ) >> 24;	// Frame interval
//	xu_data[4] = ( gH264fmt[fmt.FmtId].FrPay[fmt.FrameRateId] & 0x00FF0000 ) >> 16;
	xu_data[5] = ( gH264fmt[fmt.FmtId].FrameSize & 0x00FF0000) >> 16;
	xu_data[6] = ( gH264fmt[fmt.FmtId].FrameSize & 0x0000FF00) >> 8;
	xu_data[7] = ( gH264fmt[fmt.FmtId].FrameSize & 0x000000FF);
//	xu_data[8] = ( gH264fmt[fmt.FmtId].FrPay[fmt.FrameRateId] & 0x0000FF00 ) >> 8;
//	xu_data[9] = ( gH264fmt[fmt.FmtId].FrPay[fmt.FrameRateId] & 0x000000FF ) ;


	if(chip_id == CHIP_RER9420)
	{
		xu_data[3] = ( gH264fmt[fmt.FmtId].FrPay[fmt.FrameRateId] & 0xFF000000 ) >> 24;	// Frame interval
		xu_data[4] = ( gH264fmt[fmt.FmtId].FrPay[fmt.FrameRateId] & 0x00FF0000 ) >> 16;
		xu_data[8] = ( gH264fmt[fmt.FmtId].FrPay[fmt.FrameRateId] & 0x0000FF00 ) >> 8;
		xu_data[9] = ( gH264fmt[fmt.FmtId].FrPay[fmt.FrameRateId] & 0x000000FF ) ;
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		xu_data[3] = ( gH264fmt[fmt.FmtId].FrPay[fmt.FrameRateId*2] & 0x0000FF00 ) >> 8;	// Frame interval
		xu_data[4] = ( gH264fmt[fmt.FmtId].FrPay[fmt.FrameRateId*2] & 0x000000FF ) ;
		xu_data[8] = ( gH264fmt[fmt.FmtId].FrPay[fmt.FrameRateId*2+1] & 0xFF000000 ) >> 24;
		xu_data[9] = ( gH264fmt[fmt.FmtId].FrPay[fmt.FrameRateId*2+1] & 0x00FF0000 ) >> 16;
		xu_data[10] = ( gH264fmt[fmt.FmtId].FrPay[fmt.FrameRateId*2+1] & 0x0000FF00 ) >> 8;
	}	

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Set_FMT ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		memset(xu_data, 0, xu_size);
		xu_data[0] = ( gH264fmt[fmt.FmtId].FrPay[fmt.FrameRateId*2+1] & 0x000000FF ) ;

		if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
		{
			TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Set_FMT____2 ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
			if(err==EINVAL)
				TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
			return err;
		}
	}	

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_SetFormat <== Success \n");
	return ret;

}


int XU_H264_Get_Mode(int fd, int *Mode)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_Get_Mode ==>\n");
	int i = 0;
	int ret = 0;
	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= 0; 
	__u8 xu_selector= 0;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	if(chip_id == CHIP_RER9420)
	{
		xu_unit = XU_RERVISION_SYS_ID;
		xu_selector = XU_RERVISION_SYS_H264_CTRL;
		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x01;				// H264_ctrl_type
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		xu_unit = XU_RERVISION_USR_ID;
		xu_selector = XU_RERVISION_USR_H264_CTRL;
		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x06;				// H264_mode
	}
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Get_Mode ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	// Get mode
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Get_Mode ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	TestAp_Printf(TESTAP_DBG_FLOW, "   == XU_H264_Get_Mode Success == \n");
	TestAp_Printf(TESTAP_DBG_FLOW, "      Get data[%d] : 0x%x\n", 0, xu_data[0]);

	*Mode = xu_data[0];
	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_Get_Mode (%s)<==\n", *Mode==1?"CBR mode":(*Mode==2?"VBR mode":"error"));
	return ret;
}

int XU_H264_Set_Mode(int fd, int Mode)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_Set_Mode (0x%x) ==>\n", Mode);
	int ret = 0;
	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= 0; 
	__u8 xu_selector= 0;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	if(chip_id == CHIP_RER9420)
	{
		xu_unit = XU_RERVISION_SYS_ID;
		xu_selector = XU_RERVISION_SYS_H264_CTRL;
		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x01;				// H264_ctrl_type
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		xu_unit = XU_RERVISION_USR_ID;
		xu_selector = XU_RERVISION_USR_H264_CTRL;
		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x06;				// H264_mode
	}
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Set_Mode ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	

	// Set CBR/VBR Mode
	memset(xu_data, 0, xu_size);
	xu_data[0] = Mode;
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Set_Mode ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_Set_Mode <== Success \n");
	return ret;

}



int XU_H264_Get_QP_Limit(int fd, int *QP_Min, int *QP_Max)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_Get_QP_Limit ==>\n");
	int i = 0;
	int ret = 0;
	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= 0; 
	__u8 xu_selector= 0;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	if(chip_id == CHIP_RER9420)
	{
		xu_unit = XU_RERVISION_SYS_ID;
		xu_selector = XU_RERVISION_SYS_H264_CTRL;
		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x02;				// H264_limit
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		xu_unit = XU_RERVISION_USR_ID;
		xu_selector = XU_RERVISION_USR_H264_CTRL;
		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x01;				// H264_limit
	}


	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Get_QP_Limit ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	memset(xu_data, 0, xu_size);
	// Get QP value
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Get_QP_Limit ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	TestAp_Printf(TESTAP_DBG_FLOW, "   == XU_H264_Get_QP_Limit Success == \n");
	for(i=0; i<2; i++)
			TestAp_Printf(TESTAP_DBG_FLOW, "      Get data[%d] : 0x%x\n", i, xu_data[i]);

	*QP_Min = xu_data[0];
	*QP_Max = xu_data[1];
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_Get_QP_Limit (0x%x, 0x%x)<==\n", *QP_Min, *QP_Max);
	return ret;

}

int XU_H264_Get_QP(int fd, int *QP_Val)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_Get_QP ==>\n");
	int i = 0;
	int ret = 0;
	int err = 0;
	__u8 ctrldata[11]={0};
	*QP_Val = -1;

	if(chip_id == CHIP_RER9420)
	{
		int qp_min, qp_max;
		XU_H264_Get_QP_Limit(fd, &qp_min, &qp_max);	
	}

	//uvc_xu_control parmeters
	__u8 xu_unit= 0; 
	__u8 xu_selector= 0;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	if(chip_id == CHIP_RER9420)
	{
		xu_unit = XU_RERVISION_SYS_ID;
		xu_selector = XU_RERVISION_SYS_H264_CTRL;
		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x05;				// H264_VBR_QP
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		xu_unit = XU_RERVISION_USR_ID;
		xu_selector = XU_RERVISION_USR_H264_CTRL;
		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x07;				// H264_QP
	}
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_Get_QP ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_FLOW, "Invalid arguments\n");
		return err;
	}
	
	// Get QP value
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Get_QP ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	TestAp_Printf(TESTAP_DBG_FLOW, "   == XU_H264_Get_QP Success == \n");
	TestAp_Printf(TESTAP_DBG_FLOW, "      Get data[%d] : 0x%x\n", 0, xu_data[0]);

	*QP_Val = xu_data[0];
	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_Get_QP (0x%x)<==\n", *QP_Val);
	return ret;

}

int XU_H264_Set_QP(int fd, int QP_Val)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_Set_QP (0x%x) ==>\n", QP_Val);
	int ret = 0;
	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= 0; 
	__u8 xu_selector= 0;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	if(chip_id == CHIP_RER9420)
	{
		xu_unit = XU_RERVISION_SYS_ID;
		xu_selector = XU_RERVISION_SYS_H264_CTRL;
		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x05;				// H264_VBR_QP
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		xu_unit = XU_RERVISION_USR_ID;
		xu_selector = XU_RERVISION_USR_H264_CTRL;
		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x07;				// H264_QP	
	}
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Get_QP ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set QP value
	memset(xu_data, 0, xu_size);
	xu_data[0] = QP_Val;
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Set_QP ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_Set_QP <== Success \n");
	return ret;
}

int XU_H264_Get_BitRate(int fd, double *BitRate)
{
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_Get_BitRate ==>\n");
	int i = 0;
	int ret = 0;
	int err = 0;
	__u8 ctrldata[11]={0};
	int BitRate_CtrlNum = 0;
	*BitRate = -1.0;

	//uvc_xu_control parmeters
	__u8 xu_unit= 0; 
	__u8 xu_selector= 0;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	if(chip_id == CHIP_RER9420)
	{
		xu_unit = XU_RERVISION_SYS_ID;
		xu_selector = XU_RERVISION_SYS_H264_CTRL;
		
		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x03;				// H264_BitRate
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		xu_unit = XU_RERVISION_USR_ID;
		xu_selector = XU_RERVISION_USR_H264_CTRL;

		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x02;				// H264_BitRate
	}

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Get_BitRate ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	// Get Bit rate ctrl number
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Get_BitRate ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	TestAp_Printf(TESTAP_DBG_FLOW, "   == XU_H264_Get_BitRate Success == \n");
	
	if(chip_id == CHIP_RER9420)
	{
		for(i=0; i<2; i++)
			TestAp_Printf(TESTAP_DBG_FLOW, "      Get data[%d] : 0x%x\n", i, xu_data[i]);

		BitRate_CtrlNum = ( xu_data[0]<<8 )| (xu_data[1]) ;

		// Bit Rate = BitRate_Ctrl_Num*512*fps*8 /1000(Kbps)
		*BitRate = (double)(BitRate_CtrlNum*512.0*m_CurrentFPS*8)/1024.0;
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		for(i=0; i<3; i++)
			TestAp_Printf(TESTAP_DBG_FLOW, "      Get data[%d] : 0x%x\n", i, xu_data[i]);

		BitRate_CtrlNum =  ( xu_data[0]<<16 )| ( xu_data[1]<<8 )| (xu_data[2]) ;

		*BitRate = BitRate_CtrlNum;
	}	
	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_Get_BitRate (%.2f)<==\n", *BitRate);
	return ret;
}

int XU_H264_Set_BitRate(int fd, double BitRate)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_Set_BitRate (%.2f) ==>\n",BitRate);
	int ret = 0;
	int err = 0;
	__u8 ctrldata[11]={0};
	int BitRate_CtrlNum = 0;

	//uvc_xu_control parmeters
	__u8 xu_unit= 0; 
	__u8 xu_selector= 0;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	if(chip_id == CHIP_RER9420)
	{
		xu_unit = XU_RERVISION_SYS_ID;
		xu_selector = XU_RERVISION_SYS_H264_CTRL;

		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x03;				// H264_BitRate
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		xu_unit = XU_RERVISION_USR_ID;
		xu_selector = XU_RERVISION_USR_H264_CTRL;

		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x02;				// H264_BitRate
	}
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Set_BitRate ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set Bit Rate Ctrl Number
	if(chip_id == CHIP_RER9420)
	{
		// Bit Rate = BitRate_Ctrl_Num*512*fps*8/1000 (Kbps)
		BitRate_CtrlNum = (int)((BitRate*1024)/(512*m_CurrentFPS*8));
		xu_data[0] = (BitRate_CtrlNum & 0xFF00)>>8;	// BitRate ctrl Num
		xu_data[1] = (BitRate_CtrlNum & 0x00FF);
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		// Bit Rate = BitRate_Ctrl_Num*512*fps*8/1000 (Kbps)
		xu_data[0] = ((int)BitRate & 0x00FF0000)>>16;
		xu_data[1] = ((int)BitRate & 0x0000FF00)>>8;
		xu_data[2] = ((int)BitRate & 0x000000FF);
	}	
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Set_BitRate ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_Set_BitRate <== Success \n");
	return ret;
}

int XU_H264_Set_IFRAME(int fd)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_Set_IFRAME ==>\n");
	int ret = 0;
	int err = 0;
	__u8 ctrldata[11]={0};
	int BitRate_CtrlNum = 0;

	//uvc_xu_control parmeters
	__u8 xu_unit= 0; 
	__u8 xu_selector= 0;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	if(chip_id == CHIP_RER9420)
	{
		xu_unit = XU_RERVISION_SYS_ID;
		xu_selector = XU_RERVISION_SYS_H264_CTRL;

		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x06;				// H264_IFRAME
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		xu_unit = XU_RERVISION_USR_ID;
		xu_selector = XU_RERVISION_USR_H264_CTRL;

		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x04;				// H264_IFRAME
	}
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Set_IFRAME ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	// Set IFrame reset
	xu_data[0] = 1;
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Set_IFRAME ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}	

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_Set_IFRAME <== Success \n");
	return ret;
}

int XU_H264_Get_SEI(int fd, unsigned char *SEI)
{
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_Get_SEI ==>\n");
	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_H264_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	if(chip_id == CHIP_RER9420)
	{
		TestAp_Printf(TESTAP_DBG_FLOW, " ==SN9C290 no support get SEI==\n");	
		return 0;
	}

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x05;				// H264_SEI

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Get_SEI ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	// Get SEI
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Get_SEI ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	*SEI = xu_data[0];
	
	TestAp_Printf(TESTAP_DBG_FLOW, "      SEI : 0x%x\n",*SEI);
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_Get_SEI <== Success \n");

	return 0;
}

int XU_H264_Set_SEI(int fd, unsigned char SEI)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_Set_SEI ==>\n");
	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_H264_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	if(chip_id == CHIP_RER9420)
	{
		TestAp_Printf(TESTAP_DBG_FLOW, " ==SN9C290 no support Set SEI==\n");	
		return 0;
	}

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x05;				// H264_SEI
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Set_SEI ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set SEI
	xu_data[0] = SEI;
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Set_SEI ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_Set_SEI <== Success \n");
	return 0;
}

int XU_H264_Get_GOP(int fd, unsigned int *GOP)
{
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_Get_GOP ==>\n");
	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_H264_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x03;				// H264_GOP

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Get_GOP ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	// Get GOP
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Get_GOP ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	*GOP = (xu_data[1] << 8) | xu_data[0];
	
	TestAp_Printf(TESTAP_DBG_FLOW, "      GOP : %d\n",*GOP);
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_Get_GOP <== Success \n");

	return 0;
}

int XU_H264_Set_GOP(int fd, unsigned int GOP)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_Set_GOP ==>\n");
	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_H264_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x03;				// H264_GOP
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Set_GOP ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set GOP
	xu_data[0] = (GOP & 0xFF);
	xu_data[1] = (GOP >> 8) & 0xFF;
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_H264_Set_GOP ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_H264_Set_GOP <== Success \n");
	return 0;
}

int XU_Get(int fd, struct uvc_xu_control *xctrl)
{
	TestAp_Printf(TESTAP_DBG_FLOW, "XU Get ==>\n");
	int i = 0;
	int ret = 0;
	int err = 0;

	// XU Set
	if ((err=XU_Set_Cur(fd, xctrl->unit, xctrl->selector, xctrl->size, xctrl->data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU Get ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	// XU Get
	if ((err=XU_Get_Cur(fd, xctrl->unit, xctrl->selector, xctrl->size, xctrl->data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU Get ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	TestAp_Printf(TESTAP_DBG_FLOW, "   == XU Get Success == \n");
	for(i=0; i<xctrl->size; i++)
			TestAp_Printf(TESTAP_DBG_FLOW, "      Get data[%d] : 0x%x\n", i, xctrl->data[i]);
	return ret;
}

int XU_Set(int fd, struct uvc_xu_control xctrl)
{
	TestAp_Printf(TESTAP_DBG_FLOW, "XU Set ==>\n");
	int i = 0;
	int ret = 0;
	int err = 0;

	// XU Set
	for(i=0; i<xctrl.size; i++)
			TestAp_Printf(TESTAP_DBG_FLOW, "      Set data[%d] : 0x%x\n", i, xctrl.data[i]);
	
	if ((err=XU_Set_Cur(fd, xctrl.unit, xctrl.selector, xctrl.size, xctrl.data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU Set ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	TestAp_Printf(TESTAP_DBG_FLOW, "   == XU Set Success == \n");
	return ret;
}

int XU_Asic_Read(int fd, unsigned int Addr, unsigned char *AsicData)
{
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Asic_Read ==>\n");
	int ret = 0;
	__u8 ctrldata[4];

	//uvc_xu_control parmeters
	__u8 xu_unit= 3; 
	__u8 xu_selector= XU_RERVISION_SYS_ASIC_RW;
	__u16 xu_size= 4;
	__u8 *xu_data= ctrldata;

	xu_data[0] = (Addr & 0xFF);
	xu_data[1] = ((Addr >> 8) & 0xFF);
	xu_data[2] = 0x0;
	xu_data[3] = 0xFF;		/* Dummy Write */
	
	/* Dummy Write */
	if ((ret=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"   ioctl(UVCIOC_CTRL_SET) FAILED (%i) \n",ret);
		if(ret==EINVAL)			TestAp_Printf(TESTAP_DBG_ERR,"    Invalid arguments\n");		
		return ret;
	}
	
	/* Asic Read */
	xu_data[3] = 0x00;
	if ((ret=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"   ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",ret);
		if(ret==EINVAL)			TestAp_Printf(TESTAP_DBG_ERR,"    Invalid arguments\n");
		return ret;
	}
	*AsicData = xu_data[2];
	TestAp_Printf(TESTAP_DBG_FLOW, "   == XU_Asic_Read Success ==\n");
	TestAp_Printf(TESTAP_DBG_FLOW, "      Address:0x%x = 0x%x \n", Addr, *AsicData);
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Asic_Read <== Success\n");
	return ret;
}

int XU_Asic_Write(int fd, unsigned int Addr, unsigned char AsicData)
{
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Asic_Write ==>\n");
	int ret = 0;
	__u8 ctrldata[4];

	//uvc_xu_control parmeters
	__u8 xu_unit= 3; 
	__u8 xu_selector= XU_RERVISION_SYS_ASIC_RW;
	__u16 xu_size= 4;
	__u8 *xu_data= ctrldata;

	xu_data[0] = (Addr & 0xFF);			/* Addr Low */
	xu_data[1] = ((Addr >> 8) & 0xFF);	/* Addr High */
	xu_data[2] = AsicData;
	xu_data[3] = 0x0;					/* Normal Write */
	
	/* Normal Write */
	if ((ret=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"   ioctl(UVCIOC_CTRL_SET) FAILED (%i) \n",ret);
		if(ret==EINVAL)			TestAp_Printf(TESTAP_DBG_ERR,"    Invalid arguments\n");		
	}
	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Asic_Write <== %s\n",(ret<0?"Failed":"Success"));
	return ret;
}

int XU_Multi_Get_status(int fd, struct Multistream_Info *status)
{
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Get_status ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_MULTI_STREAM_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x01;				// Multi-Stream Status
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Get_status ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get status
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Get_status ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	status->strm_type = xu_data[0];
	status->format = ( xu_data[1]<<8 )| (xu_data[2]) ;

	TestAp_Printf(TESTAP_DBG_FLOW, "   == XU_Multi_Get_status Success == \n");
	TestAp_Printf(TESTAP_DBG_FLOW, "      Get strm_type %d\n", status->strm_type);
	TestAp_Printf(TESTAP_DBG_FLOW, "      Get cur_format %d\n", status->format);

	return 0;
}

int XU_Multi_Get_Info(int fd, struct Multistream_Info *Info)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Get_Info ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_MULTI_STREAM_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x02;				// Multi-Stream Stream Info
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Get_Info ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get Info
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Get_Info ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	Info->strm_type = xu_data[0];
	Info->format = ( xu_data[1]<<8 )| (xu_data[2]) ;

	TestAp_Printf(TESTAP_DBG_FLOW, "   == XU_Multi_Get_Info Success == \n");
	TestAp_Printf(TESTAP_DBG_FLOW, "      Get Support Stream %d\n", Info->strm_type);
	TestAp_Printf(TESTAP_DBG_FLOW, "      Get Support Resolution 0x%x\n", Info->format);

	return 0;
}

int XU_Multi_Set_Type(int fd, unsigned int format)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Set_Type (%d) ==>\n",format);

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_MULTI_STREAM_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x02;
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Set_Type ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	if(chip_id == CHIP_RER9421 && (format==4 ||format==8 ||format==16))
	{
		TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return -1;
	}
	

	// Set format
	xu_data[0] = format;
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Set_Type ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Set_Type <== Success \n");
	return 0;

}

int XU_Multi_Set_Enable(int fd, unsigned char enable)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Set_Enable ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};
	int BitRate_CtrlNum = 0;

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_MULTI_STREAM_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x03;				// Enable Multi-Stream Flag

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Set_Enable ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set enable / disable multistream
	xu_data[0] = enable;
	xu_data[1] = 0;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Set_Enable ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	TestAp_Printf(TESTAP_DBG_FLOW, "Set H264_Multi_Enable = %d \n",(xu_data[0] &0x01));
	TestAp_Printf(TESTAP_DBG_FLOW, "Set MJPG_Multi_Enable = %d \n",((xu_data[0] >> 1) &0x01));
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Set_Enable <== Success \n");
	return 0;
}

int XU_Multi_Get_Enable(int fd, unsigned char *enable)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Get_Enable ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_MULTI_STREAM_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x03;				// Enable Multi-Stream Flag
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Get_Enable ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get Enable
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Get_Enable ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	*enable = xu_data[0];

	TestAp_Printf(TESTAP_DBG_FLOW, "      Get H264 Multi Stream Enable = %d\n", xu_data[0] & 0x01);
	TestAp_Printf(TESTAP_DBG_FLOW, "      Get MJPG Multi Stream Enable =  %d\n", (xu_data[0] >> 1) & 0x01);	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Get_Enable <== Success\n");
	return 0;
}

#if 0
int XU_Multi_Set_BitRate(int fd, unsigned int BitRate1, unsigned int BitRate2, unsigned int BitRate3)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Set_BitRate  BiteRate1=%d  BiteRate2=%d  BiteRate3=%d   ==>\n",BitRate1, BitRate2, BitRate3);

	int err = 0;
	__u8 ctrldata[11]={0};
	int BitRate_CtrlNum = 0;

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_MULTI_STREAM_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;
	xu_data[1] = 0x04;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Set_BitRate ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set BitRate1~3  (unit:bps)
	xu_data[0] = (BitRate1 >> 16)&0xFF;
	xu_data[1] = (BitRate1 >> 8)&0xFF;
	xu_data[2] = BitRate1&0xFF;
	xu_data[3] = (BitRate2 >> 16)&0xFF;
	xu_data[4] = (BitRate2 >> 8)&0xFF;
	xu_data[5] = BitRate2&0xFF;
	xu_data[6] = (BitRate3 >> 16)&0xFF;
	xu_data[7] = (BitRate3 >> 8)&0xFF;
	xu_data[8] = BitRate3&0xFF;
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Set_BitRate ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Set_BitRate <== Success \n");
	return 0;
}

int XU_Multi_Get_BitRate(int fd)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Get_BitRate  ==>\n");

	int i = 0;
	int err = 0;
	int BitRate1 = 0;
	int BitRate2 = 0;
	int BitRate3 = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_MULTI_STREAM_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;
	xu_data[1] = 0x04;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Get_BitRate ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get BitRate1~3  (unit:bps)
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Get_BitRate ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Get_BitRate <== Success \n");
	
	for(i=0; i<9; i++)
		TestAp_Printf(TESTAP_DBG_FLOW, "      Get data[%d] : 0x%x\n", i, xu_data[i]);

	BitRate1 =  ( xu_data[0]<<16 )| ( xu_data[1]<<8 )| (xu_data[2]) ;
	BitRate2 =  ( xu_data[3]<<16 )| ( xu_data[4]<<8 )| (xu_data[5]) ;
	BitRate3 =  ( xu_data[6]<<16 )| ( xu_data[7]<<8 )| (xu_data[8]) ;
	
	TestAp_Printf(TESTAP_DBG_FLOW, "  HD BitRate (%d)\n", BitRate1);
	TestAp_Printf(TESTAP_DBG_FLOW, "  QVGA BitRate (%d)\n", BitRate2);
	TestAp_Printf(TESTAP_DBG_FLOW, "  QQVGA BitRate (%d)\n", BitRate3);

	return 0;
}
#endif

int XU_Multi_Set_BitRate(int fd, unsigned int StreamID, unsigned int BitRate)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Set_BitRate  StreamID=%d  BiteRate=%d  ==>\n",StreamID ,BitRate);

	int err = 0;
	__u8 ctrldata[11]={0};
	int BitRate_CtrlNum = 0;

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_MULTI_STREAM_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;
	xu_data[1] = 0x04;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Set_BitRate ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set BitRate  (unit:bps)
	xu_data[0] = StreamID;
	xu_data[1] = (BitRate >> 16)&0xFF;
	xu_data[2] = (BitRate >> 8)&0xFF;
	xu_data[3] = BitRate&0xFF;
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Set_BitRate ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Set_BitRate <== Success \n");
	return 0;
}

int XU_Multi_Get_BitRate(int fd, unsigned int StreamID, unsigned int *BitRate)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Get_BitRate  ==>\n");

	int i = 0;
	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_MULTI_STREAM_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;
	xu_data[1] = 0x05;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Get_BitRate ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set Stream ID
	xu_data[0] = StreamID;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Get_BitRate ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get BitRate (unit:bps)
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Get_BitRate ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Get_BitRate <== Success \n");
	
	for(i=0; i<4; i++)
		TestAp_Printf(TESTAP_DBG_FLOW, "      Get data[%d] : 0x%x\n", i, xu_data[i]);

	*BitRate = ( xu_data[1]<<16 ) | ( xu_data[2]<<8 ) | xu_data[3];
	TestAp_Printf(TESTAP_DBG_FLOW, "  Stream= %d   BitRate= %d\n", xu_data[0], *BitRate);

	return 0;
}

int XU_Multi_Set_QP(int fd, unsigned int StreamID, unsigned int QP_Val)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Set_QP  StreamID=%d  QP_Val=%d  ==>\n",StreamID ,QP_Val);

	int err = 0;
	__u8 ctrldata[11]={0};
	int BitRate_CtrlNum = 0;

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_MULTI_STREAM_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;
	xu_data[1] = 0x05;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Set_QP ==> Switch cmd(5) : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set Stream ID
	xu_data[0] = StreamID;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Set_QP ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Switch command
	xu_data[0] = 0x9A;
	xu_data[1] = 0x06;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Set_QP ==> Switch cmd(6) : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set QP
	xu_data[0] = StreamID;
	xu_data[1] = QP_Val;

	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Set_QP ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Set_QP <== Success \n");
	return 0;
}

int XU_Multi_Get_QP(int fd, unsigned int StreamID, unsigned int *QP_val)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Get_QP  ==>\n");

	int i = 0;
	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_MULTI_STREAM_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;
	xu_data[1] = 0x05;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Get_QP ==> Switch cmd(5) : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set Stream ID
	xu_data[0] = StreamID;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Get_QP ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Switch command
	xu_data[0] = 0x9A;
	xu_data[1] = 0x06;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Get_QP ==> Switch cmd(6) : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get QP
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Get_QP ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Get_QP <== Success \n");
	
	for(i=0; i<2; i++)
		TestAp_Printf(TESTAP_DBG_FLOW, "      Get data[%d] : 0x%x\n", i, xu_data[i]);

	*QP_val = xu_data[1];
	TestAp_Printf(TESTAP_DBG_FLOW, "  Stream= %d   QP_val = %d\n", xu_data[0], *QP_val);

	return 0;
}

int XU_Multi_Set_H264Mode(int fd, unsigned int StreamID, unsigned int Mode)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Set_H264Mode  StreamID=%d  Mode=%d  ==>\n",StreamID ,Mode);

	int err = 0;
	__u8 ctrldata[11]={0};
	int BitRate_CtrlNum = 0;

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_MULTI_STREAM_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;
	xu_data[1] = 0x07;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Set_H264Mode ==> Switch cmd(7) : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set Stream ID
	xu_data[0] = StreamID;
    xu_data[1] = Mode;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Set_H264Mode ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Set_H264Mode <== Success \n");
	return 0;
}

int XU_Multi_Get_H264Mode(int fd, unsigned int StreamID, unsigned int *Mode)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Get_H264Mode  ==>\n");

	int i = 0;
	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_MULTI_STREAM_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;
	xu_data[1] = 0x05;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Get_H264Mode ==> Switch cmd(5) : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set Stream ID
	xu_data[0] = StreamID;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Get_H264Mode ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Switch command
	xu_data[0] = 0x9A;
	xu_data[1] = 0x07;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Get_H264Mode ==> Switch cmd(7) : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get H264 Mode
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Get_H264Mode ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Get_H264Mode <== Success \n");
	
	for(i=0; i<2; i++)
		TestAp_Printf(TESTAP_DBG_FLOW, "      Get data[%d] : 0x%x\n", i, xu_data[i]);

	*Mode = xu_data[1];
	TestAp_Printf(TESTAP_DBG_FLOW, "  Stream= %d   Mode = %d\n", xu_data[0], *Mode);

	return 0;
}

int XU_Multi_Set_SubStream_FrameRate(int fd, unsigned int sub_fps)
{   
    TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Set_SubStream_FrameRate sub_fps=%d  ==>\n",sub_fps);

    int err = 0, main_fps = 0;
    __u8 ctrldata[11]={0};
    int BitRate_CtrlNum = 0;

    //uvc_xu_control parmeters
    __u8 xu_unit= XU_RERVISION_USR_ID; 
    __u8 xu_selector= XU_RERVISION_USR_MULTI_STREAM_CTRL;
    __u16 xu_size= 11;
    __u8 *xu_data= ctrldata;

    video_get_framerate(fd, &main_fps);
    if(sub_fps>main_fps)
    {
        TestAp_Printf(TESTAP_DBG_ERR,"set sub_fps as %d, because sub_fps must less than or equal to main_fps\n", main_fps);
        sub_fps = main_fps;
    }

    // Switch command
    xu_data[0] = 0x9A;
    xu_data[1] = 0x08;

    if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
    {
        TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Set_SubStream_FrameRate ==> Switch cmd(8) : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
        if(err==EINVAL)
            TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
        return err;
    }

    // Set Stream ID
    xu_data[0] = sub_fps;

    if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
    {
        TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Set_SubStream_FrameRate ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
        if(err==EINVAL)
            TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
        return err;
    }

    TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Set_SubStream_FrameRate <== Success \n");
    return 0;
}

int XU_Multi_Get_SubStream_FrameRate(int fd, unsigned int *sub_fps)

{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Get_SubStream_FrameRate  ==>\n");

	int i = 0;
	int err = 0, main_fps;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_MULTI_STREAM_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;
    
    video_get_framerate(fd, &main_fps);
    
	// Switch command
	xu_data[0] = 0x9A;
	xu_data[1] = 0x08;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Get_H264Mode ==> Switch cmd(8) : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get substream fr
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Get_SubStream_FrameRate ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Get_SubStream_FrameRate <== Success \n");
	

	*sub_fps = (xu_data[0]<main_fps?xu_data[0]:main_fps);
	TestAp_Printf(TESTAP_DBG_FLOW, "sub_fps = min(%d, %d)\n",  xu_data[0],main_fps);

	return 0;
}

int XU_Multi_Set_SubStream_GOP(int fd, unsigned int sub_gop)
{   
    TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Set_SubStream_GOP sub_gop=%d  ==>\n",sub_gop);

    int err = 0, main_gop = 0;
    __u8 ctrldata[11]={0};
    int BitRate_CtrlNum = 0;

    //uvc_xu_control parmeters
    __u8 xu_unit= XU_RERVISION_USR_ID; 
    __u8 xu_selector= XU_RERVISION_USR_MULTI_STREAM_CTRL;
    __u16 xu_size= 11;
    __u8 *xu_data= ctrldata;

    XU_H264_Get_GOP(fd, &main_gop);
    if(sub_gop>main_gop)
    {
        TestAp_Printf(TESTAP_DBG_ERR,"set sub_gop as %d, because sub_gop must less than or equal to main_gop\n", main_gop);
        sub_gop= main_gop;
    }

    // Switch command
    xu_data[0] = 0x9A;
    xu_data[1] = 0x09;

    if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
    {
        TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Set_SubStream_GOP ==> Switch cmd(9) : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
        if(err==EINVAL)
            TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
        return err;
    }

    // Set sub_gop
    xu_data[0] = sub_gop&0xff;
    xu_data[1] = (sub_gop&0xff00)>>8;
    if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
    {
        TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Set_SubStream_GOP ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
        if(err==EINVAL)
            TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
        return err;
    }

    TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Set_SubStream_GOP <== Success \n");
    return 0;
}

int XU_Multi_Get_SubStream_GOP(int fd, unsigned int *sub_gop)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Get_SubStream_GOP  ==>\n");

	int i = 0;
	int err = 0, main_gop, get_gop;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_MULTI_STREAM_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;
    
    XU_H264_Get_GOP(fd, &main_gop);
    
	// Switch command
	xu_data[0] = 0x9A;
	xu_data[1] = 0x09;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Get_SubStream_GOP ==> Switch cmd(9) : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get substream gop
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Multi_Get_SubStream_GOP ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Multi_Get_SubStream_GOP <== Success \n");
	get_gop = (xu_data[1]<<8) | xu_data[0]; 
	*sub_gop= (get_gop<main_gop?get_gop:main_gop);
	TestAp_Printf(TESTAP_DBG_FLOW, "sub_fps = min(%d, %d)\n",  get_gop,main_gop);

	return 0;
}


int XU_OSD_Timer_Ctrl(int fd, unsigned char enable)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Timer_Ctrl  ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x00;				// OSD Timer control

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Timer_Ctrl ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set enable / disable timer count
	xu_data[0] = enable;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Timer_Ctrl ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Timer_Ctrl <== Success \n");

	return 0;
}

int XU_OSD_Set_RTC(int fd, unsigned int year, unsigned char month, unsigned char day, unsigned char hour, unsigned char minute, unsigned char second)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_RTC  ==>\n");

	int i = 0;
	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x01;				// OSD RTC control

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_RTC ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set RTC
	xu_data[0] = second;
	xu_data[1] = minute;
	xu_data[2] = hour;
	xu_data[3] = day;
	xu_data[4] = month;
	xu_data[5] = (year & 0xFF00) >> 8;
	xu_data[6] = (year & 0x00FF);

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_RTC ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	for(i=0; i<7; i++)
		TestAp_Printf(TESTAP_DBG_FLOW, "      Set data[%d] : 0x%x\n", i, xu_data[i]);
	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_RTC <== Success \n");

	return 0;
}

int XU_OSD_Get_RTC(int fd, unsigned int *year, unsigned char *month, unsigned char *day, unsigned char *hour, unsigned char *minute, unsigned char *second)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_RTC  ==>\n");

	int i = 0;
	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x01;				// OSD RTC control

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Get_RTC ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get data
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0)
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Get_RTC ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	for(i=0; i<7; i++)
		TestAp_Printf(TESTAP_DBG_FLOW, "      Get data[%d] : 0x%x\n", i, xu_data[i]);

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_RTC <== Success \n");
	
	*year = (xu_data[5]<<8) | xu_data[6];
	*month = xu_data[4];
	*day = xu_data[3];
	*hour = xu_data[2];
	*minute = xu_data[1];
	*second = xu_data[0];

	TestAp_Printf(TESTAP_DBG_FLOW, " year 	= %d \n",*year);
	TestAp_Printf(TESTAP_DBG_FLOW, " month	= %d \n",*month);
	TestAp_Printf(TESTAP_DBG_FLOW, " day 	= %d \n",*day);
	TestAp_Printf(TESTAP_DBG_FLOW, " hour 	= %d \n",*hour);
	TestAp_Printf(TESTAP_DBG_FLOW, " minute	= %d \n",*minute);
	TestAp_Printf(TESTAP_DBG_FLOW, " second	= %d \n",*second);
	
	return 0;
}

int XU_OSD_Set_Size(int fd, unsigned char LineSize, unsigned char BlockSize)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_Size  ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x02;				// OSD Size control

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_Size ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	if(LineSize > 4)
		LineSize = 4;

	if(BlockSize > 4)
		BlockSize = 4;
		
	// Set data
	xu_data[0] = LineSize;
	xu_data[1] = BlockSize;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_Size ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_Size <== Success \n");

	return 0;
}

int XU_OSD_Get_Size(int fd, unsigned char *LineSize, unsigned char *BlockSize)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_Size  ==>\n");

	int i = 0;
	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x02;				// OSD Size control

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Get_Size ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get data
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0)
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Get_Size ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	for(i=0; i<2; i++)
		TestAp_Printf(TESTAP_DBG_FLOW, "      Get data[%d] : 0x%x\n", i, xu_data[i]);

	*LineSize = xu_data[0];
	*BlockSize = xu_data[1];

	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Size (Line) = %d\n",*LineSize);
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Size (Block) = %d\n",*BlockSize);
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_Size <== Success \n");
	
	return 0;
}

int XU_OSD_Set_Color(int fd, unsigned char FontColor, unsigned char BorderColor)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_Color  ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x03;				// OSD Color control

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_Color ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	if(FontColor > 4)
		FontColor = 4;

	if(BorderColor > 4)
		BorderColor = 4;
		
	// Set data
	xu_data[0] = FontColor;
	xu_data[1] = BorderColor;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_Color ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_Color <== Success \n");

	return 0;
}

int XU_OSD_Get_Color(int fd, unsigned char *FontColor, unsigned char *BorderColor)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_Color  ==>\n");

	int i = 0;
	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x03;				// OSD Color control

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Get_Color ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get data
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0)
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Get_Color ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	for(i=0; i<2; i++)
		TestAp_Printf(TESTAP_DBG_FLOW, "      Get data[%d] : 0x%x\n", i, xu_data[i]);

	*FontColor = xu_data[0];
	*BorderColor = xu_data[1];

	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Font Color = %d\n",*FontColor );
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Border Color = %d\n",*BorderColor);
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_Color <== Success \n");
	
	return 0;
}

int XU_OSD_Set_Enable(int fd, unsigned char Enable_Line, unsigned char Enable_Block)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_Enable  ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x04;				// OSD enable

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_Enable ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
		
	// Set data
	xu_data[0] = Enable_Line;
	xu_data[1] = Enable_Block;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_Enable ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_Enable <== Success \n");

	return 0;
}

int XU_OSD_Get_Enable(int fd, unsigned char *Enable_Line, unsigned char *Enable_Block)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_Enable  ==>\n");

	int i = 0;
	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x04;				// OSD Enable

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Get_Enable ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get data
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0)
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Get_Enable ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	for(i=0; i<2; i++)
		TestAp_Printf(TESTAP_DBG_FLOW, "      Get data[%d] : 0x%x\n", i, xu_data[i]);

	*Enable_Line = xu_data[0];
	*Enable_Block = xu_data[1];
	
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Enable Line = %d\n",*Enable_Line);
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Enable Block = %d\n",*Enable_Block);

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_Enable <== Success \n");
	
	return 0;
}

int XU_OSD_Set_AutoScale(int fd, unsigned char Enable_Line, unsigned char Enable_Block)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_AutoScale  ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x05;				// OSD Auto Scale enable

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_AutoScale ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
		
	// Set data
	xu_data[0] = Enable_Line;
	xu_data[1] = Enable_Block;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_AutoScale ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_AutoScale <== Success \n");

	return 0;
}

int XU_OSD_Get_AutoScale(int fd, unsigned char *Enable_Line, unsigned char *Enable_Block)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_AutoScale  ==>\n");

	int i = 0;
	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x05;				// OSD Auto Scale enable

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Get_AutoScale ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get data
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0)
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Get_AutoScale ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	for(i=0; i<2; i++)
		TestAp_Printf(TESTAP_DBG_FLOW, "      Get data[%d] : 0x%x\n", i, xu_data[i]);

	*Enable_Line = xu_data[0];
	*Enable_Block = xu_data[1];

	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Enable Line  Auto Scale = %d\n",*Enable_Line);
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Enable Block Auto Scale = %d\n",*Enable_Block);
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_AutoScale <== Success \n");
	
	return 0;
}

int XU_OSD_Set_Multi_Size(int fd, unsigned char Stream0, unsigned char Stream1, unsigned char Stream2)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_Multi_Size  %d   %d   %d  ==>\n",Stream0 ,Stream1 , Stream2);

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x06;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_Multi_Size ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	// Set data
	xu_data[0] = Stream0;
	xu_data[1] = Stream1;
	xu_data[2] = Stream2;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_Multi_Size ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_Multi_Size <== Success \n");

	return 0;
}

int XU_OSD_Get_Multi_Size(int fd, unsigned char *Stream0, unsigned char *Stream1, unsigned char *Stream2)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_Multi_Size  ==>\n");

	int i = 0;
	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x06;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Get_Multi_Size ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get data
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0)
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Get_Multi_Size ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	for(i=0; i<3; i++)
		TestAp_Printf(TESTAP_DBG_FLOW, "      Get data[%d] : 0x%x\n", i, xu_data[i]);

	*Stream0 = xu_data[0];
	*Stream1 = xu_data[1];
	*Stream2 = xu_data[2];
	
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Multi Stream 0 Size = %d\n",*Stream0);
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Multi Stream 1 Size = %d\n",*Stream1);
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Multi Stream 2 Size = %d\n",*Stream2);
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_Multi_Size <== Success \n");
	
	return 0;
}

int XU_OSD_Set_Start_Position(int fd, unsigned char OSD_Type, unsigned int RowStart, unsigned int ColStart)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_Start_Position  ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x08;				// OSD Start Position

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_Start_Position ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	if(OSD_Type > 3)
		OSD_Type = 0;
		
	// Set data
	xu_data[0] = OSD_Type;
	xu_data[1] = (RowStart & 0xFF00) >> 8;	//unit 16 lines
	xu_data[2] = RowStart & 0x00FF;
	xu_data[3] = (ColStart & 0xFF00) >> 8;	//unit 16 pixels
	xu_data[4] = ColStart & 0x00FF;	

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_Start_Position ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_Start_Position <== Success \n");

	return 0;
}

int XU_OSD_Get_Start_Position(int fd, unsigned int *LineRowStart, unsigned int *LineColStart, unsigned int *BlockRowStart, unsigned int *BlockColStart)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_Start_Position  ==>\n");

	int i = 0;
	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x08;				// OSD Start Position

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_Start_Position ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get data
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0)
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_Start_Position ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	for(i=0; i<8; i++)
		TestAp_Printf(TESTAP_DBG_FLOW, "      Get data[%d] : 0x%x\n", i, xu_data[i]);

	*LineRowStart = (xu_data[0] << 8) | xu_data[1];
	*LineColStart = (xu_data[2] << 8) | xu_data[3];
	*BlockRowStart = (xu_data[4] << 8) | xu_data[5];
	*BlockColStart = (xu_data[6] << 8) | xu_data[7];
	
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Line Start Row =%d * 16lines\n",*LineRowStart);
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Line Start Col =%d * 16pixels\n",*LineColStart);
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Block Start Row =%d * 16lines\n",*BlockRowStart);
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Block Start Col =%d * 16pixels\n",*BlockColStart);
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_Start_Position <== Success \n");
	
	return 0;
}

int XU_OSD_Set_MS_Start_Position(int fd, unsigned char StreamID, unsigned char RowStart, unsigned char ColStart)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_MS_Start_Position  ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x09;				// OSD MS Start Position

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_MS_Start_Position ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	// Set data
	xu_data[0] = StreamID;
	xu_data[1] = RowStart & 0x00FF;
	xu_data[2] = ColStart & 0x00FF;	

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_MS_Start_Position ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_MS_Start_Position  %d %d %d<== Success \n", StreamID, RowStart, ColStart);

	return 0;
}

int XU_OSD_Get_MS_Start_Position(int fd, unsigned char *S0_Row, unsigned char *S0_Col, unsigned char *S1_Row, unsigned char *S1_Col, unsigned char *S2_Row, unsigned char *S2_Col)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_MS_Start_Position  ==>\n");

	int i = 0;
	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x09;				// OSD MS Start Position

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Get_MS_Start_Position ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get data
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0)
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Get_MS_Start_Position ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	for(i=0; i<6; i++)
		TestAp_Printf(TESTAP_DBG_FLOW, "      Get data[%d] : 0x%x\n", i, xu_data[i]);

	*S0_Row = xu_data[0];
	*S0_Col = xu_data[1];
	*S1_Row = xu_data[2];
	*S1_Col = xu_data[3];
	*S2_Row = xu_data[4];
	*S2_Col = xu_data[5];
	
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Stream0 Start Row = %d * 16lines\n",*S0_Row);
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Stream0 Start Col = %d * 16pixels\n",*S0_Col);
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Stream1 Start Row = %d * 16lines\n",*S1_Row);
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Stream1 Start Col = %d * 16pixels\n",*S1_Col);
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Stream2 Start Row = %d * 16lines\n",*S2_Row);
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Stream2 Start Col = %d * 16pixels\n",*S2_Col);
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_MS_Start_Position <== Success \n");
	
	return 0;
}

int XU_OSD_Set_String(int fd, unsigned char group, char *String)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_String  ==>\n");

	int i = 0;
	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x07;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_String ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set data
	xu_data[0] = 1;
	xu_data[1] = group;

	for(i=0; i<8; i++)
		xu_data[i+2] = String[i];

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_String ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_String <== Success \n");
	
	return 0;
}

int XU_OSD_Get_String(int fd, unsigned char group, char *String)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_String  ==>\n");

	int i = 0;
	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x07;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Get_String ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set read mode
	xu_data[0] = 0;
	xu_data[1] = group;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_String ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get data
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0)
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Get_String ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	group = xu_data[1];

	TestAp_Printf(TESTAP_DBG_FLOW, "      Get data[0] : 0x%x\n", xu_data[0]);
	TestAp_Printf(TESTAP_DBG_FLOW, "      Get data[1] : 0x%x\n", group);
	
	for(i=0; i<8; i++)
	{
		String[i] = xu_data[i+2];
		TestAp_Printf(TESTAP_DBG_FLOW, "      Get data[%d] : 0x%x\n", i+2, String[i]);
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "OSD String = %s \n",String);
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_String <== Success \n");

	return 0;
}

int XU_MD_Set_Mode(int fd, unsigned char Enable)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_MD_Set_Mode  ==>\n");

	int err = 0;
	__u8 ctrldata[24]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_MOTION_DETECTION;
	__u16 xu_size= 24;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x01;				// Motion detection mode

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_MD_Set_Mode ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set data
	xu_data[0] = Enable;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_MD_Set_Mode ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_MD_Set_Mode <== Success \n");

	return 0;
}

int XU_MD_Get_Mode(int fd, unsigned char *Enable)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_MD_Get_Mode  ==>\n");

	int err = 0;
	__u8 ctrldata[24]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_MOTION_DETECTION;
	__u16 xu_size= 24;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x01;				// Motion detection mode

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_MD_Get_Mode ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get data
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0)
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_MD_Get_Mode ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	*Enable = xu_data[0];

	TestAp_Printf(TESTAP_DBG_FLOW, "Motion Detect mode = %d\n",*Enable);
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_MD_Get_Mode <== Success \n");
	
	return 0;
}

int XU_MD_Set_Threshold(int fd, unsigned int MD_Threshold)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_MD_Set_Threshold  ==>\n");

	int err = 0;
	__u8 ctrldata[24]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_MOTION_DETECTION;
	__u16 xu_size= 24;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x02;				// Motion detection threshold

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_MD_Set_Threshold ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set data
	xu_data[0] = (MD_Threshold & 0xFF00) >> 8;
	xu_data[1] = MD_Threshold & 0x00FF;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_MD_Set_Threshold ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_MD_Set_Threshold <== Success \n");

	return 0;
}

int XU_MD_Get_Threshold(int fd, unsigned int *MD_Threshold)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_MD_Get_Threshold  ==>\n");

	int err = 0;
	__u8 ctrldata[24]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_MOTION_DETECTION;
	__u16 xu_size= 24;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x02;				// Motion detection threshold

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_MD_Get_Threshold ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get data
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0)
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_MD_Get_Threshold ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	*MD_Threshold = (xu_data[0] << 8) | xu_data[1];
	
	TestAp_Printf(TESTAP_DBG_FLOW, "Motion Detect threshold = %d\n",*MD_Threshold);
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_MD_Get_Threshold <== Success \n");
	
	return 0;
}

int XU_MD_Set_Mask(int fd, unsigned char *Mask)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_MD_Set_Mask  ==>\n");

	int err = 0;
	unsigned char i;
	__u8 ctrldata[24]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_MOTION_DETECTION;
	__u16 xu_size= 24;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x03;				// Motion detection mask

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_MD_Set_Mask ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set data
	for(i=0; i < 24; i++)
	{
		xu_data[i] = Mask[i];
	}

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_MD_Set_Mask ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_MD_Set_Mask <== Success \n");

	return 0;
}

int XU_MD_Get_Mask(int fd, unsigned char *Mask)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_MD_Get_Mask  ==>\n");

	int err = 0;
	int i,j,k,l;
	__u8 ctrldata[24]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_MOTION_DETECTION;
	__u16 xu_size= 24;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x03;				// Motion detection mask

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_MD_Get_Mask ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)  \n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get data
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0)
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_MD_Get_Mask ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i) \n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	for(i=0; i<24; i++)
	{
		TestAp_Printf(TESTAP_DBG_FLOW, "      Get data[%d] : 0x%x\n", i, xu_data[i]);
		Mask[i] = xu_data[i];
	}
	
	TestAp_Printf(TESTAP_DBG_FLOW, "               ======   Motion Detect Mask   ======                \n");
	TestAp_Printf(TESTAP_DBG_FLOW, "     1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16 \n");
	
	for(k=0; k<12; k++)
	{
		TestAp_Printf(TESTAP_DBG_FLOW, "%2d   ",k+1);
		for(j=0; j<2; j++)
		{
			for(i=0; i<8; i++)
				TestAp_Printf(TESTAP_DBG_FLOW, "%d   ",(Mask[k*2+j]>>i)&0x01);
		}
		TestAp_Printf(TESTAP_DBG_FLOW, "\n");
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_MD_Get_Mask <== Success \n");
	
	return 0;
}

int XU_MD_Set_RESULT(int fd, unsigned char *Result)
{	
	//TestAp_Printf(TESTAP_DBG_FLOW, "XU_MD_Set_RESULT  ==>\n");

	int err = 0;
	unsigned char i;
	__u8 ctrldata[24]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_MOTION_DETECTION;
	__u16 xu_size= 24;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x04;				// Motion detection Result

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_MD_Set_RESULT ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set data
	for(i=0; i < 24; i++)
	{
		xu_data[i] = Result[i];
	}

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_MD_Set_RESULT ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	//TestAp_Printf(TESTAP_DBG_FLOW, "XU_MD_Set_RESULT <== Success \n");

	return 0;
}

int XU_MD_Get_RESULT(int fd, unsigned char *Result)
{	
	//TestAp_Printf(TESTAP_DBG_FLOW, "XU_MD_Get_RESULT  ==>\n");

	int err = 0;
	int i,j,k,l;
	__u8 ctrldata[24]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_MOTION_DETECTION;
	__u16 xu_size= 24;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x04;				// Motion detection Result

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_MD_Get_RESULT ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)  \n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get data
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0)
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_MD_Get_RESULT ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i) \n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	for(i=0; i<24; i++)
	{
		//TestAp_Printf(TESTAP_DBG_FLOW, "      Get data[%d] : 0x%x\n", i, xu_data[i]);
		Result[i] = xu_data[i];
	}

	system("clear");
	TestAp_Printf(TESTAP_DBG_FLOW, "               ------   Motion Detect Result   ------                \n");
	TestAp_Printf(TESTAP_DBG_FLOW, "     1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16 \n");
	
	for(k=0; k<12; k++)
	{
		TestAp_Printf(TESTAP_DBG_FLOW, "%2d   ",k+1);
		for(j=0; j<2; j++)
		{
			for(i=0; i<8; i++)
				TestAp_Printf(TESTAP_DBG_FLOW, "%d   ",(Result[k*2+j]>>i)&0x01);
		}
		TestAp_Printf(TESTAP_DBG_FLOW, "\n");
	}

	//TestAp_Printf(TESTAP_DBG_FLOW, "XU_MD_Get_RESULT <== Success \n");
	
	return 0;
}

int XU_MJPG_Get_Bitrate(int fd, unsigned int *MJPG_Bitrate)
{
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_MJPG_Get_Bitrate ==>\n");
	int i = 0;
	int ret = 0;
	int err = 0;
	__u8 ctrldata[11]={0};
	int BitRate_CtrlNum = 0;
	*MJPG_Bitrate = 0;

	//uvc_xu_control parmeters
	__u8 xu_unit= 0; 
	__u8 xu_selector= 0;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	if(chip_id == CHIP_RER9420)
	{
		xu_unit = XU_RERVISION_SYS_ID;
		xu_selector = XU_RERVISION_SYS_MJPG_CTRL;
		
		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x02;
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		xu_unit = XU_RERVISION_USR_ID;
		xu_selector = XU_RERVISION_USR_MJPG_CTRL;

		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x01;
	}

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_MJPG_Get_Bitrate ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	// Get Bit rate ctrl number
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_MJPG_Get_Bitrate ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	TestAp_Printf(TESTAP_DBG_FLOW, "   == XU_MJPG_Get_Bitrate Success == \n");

	if(chip_id == CHIP_RER9420)
	{
		for(i=0; i<2; i++)
			TestAp_Printf(TESTAP_DBG_FLOW, "      Get data[%d] : 0x%x\n", i, xu_data[i]);

		BitRate_CtrlNum = ( xu_data[0]<<8 )| (xu_data[1]) ;

		// Bit Rate = BitRate_Ctrl_Num*256*fps*8 /1024(Kbps)
		*MJPG_Bitrate = (BitRate_CtrlNum*256.0*m_CurrentFPS*8)/1024.0;
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		for(i=0; i<4; i++)
			TestAp_Printf(TESTAP_DBG_FLOW, "      Get data[%d] : 0x%x\n", i, xu_data[i]);

		*MJPG_Bitrate = (xu_data[0] << 24) | (xu_data[1] << 16) | (xu_data[2] << 8) | (xu_data[3]) ;
	}
	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_MJPG_Get_Bitrate (%x)<==\n", *MJPG_Bitrate);
	return ret;
}

int XU_MJPG_Set_Bitrate(int fd, unsigned int MJPG_Bitrate)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_MJPG_Set_Bitrate (%x) ==>\n",MJPG_Bitrate);
	int ret = 0;
	int err = 0;
	__u8 ctrldata[11]={0};
	int BitRate_CtrlNum = 0;

	//uvc_xu_control parmeters
	__u8 xu_unit= 0; 
	__u8 xu_selector= 0;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	if(chip_id == CHIP_RER9420)
	{
		xu_unit = XU_RERVISION_SYS_ID;
		xu_selector = XU_RERVISION_SYS_MJPG_CTRL;
		
		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x02;
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		xu_unit = XU_RERVISION_USR_ID;
		xu_selector = XU_RERVISION_USR_MJPG_CTRL;

		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x01;
	}
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_MJPG_Set_Bitrate ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set Bit Rate Ctrl Number
	if(chip_id == CHIP_RER9420)
	{
		// Bit Rate = BitRate_Ctrl_Num*256*fps*8/1024 (Kbps)
		BitRate_CtrlNum = ((MJPG_Bitrate*1024)/(256*m_CurrentFPS*8));

		xu_data[0] = (BitRate_CtrlNum & 0xFF00) >> 8;
		xu_data[1] = (BitRate_CtrlNum & 0x00FF);
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		xu_data[0] = (MJPG_Bitrate & 0xFF000000) >> 24;
		xu_data[1] = (MJPG_Bitrate & 0x00FF0000) >> 16;
		xu_data[2] = (MJPG_Bitrate & 0x0000FF00) >> 8;
		xu_data[3] = (MJPG_Bitrate & 0x000000FF);
	}

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_MJPG_Set_Bitrate ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_MJPG_Set_Bitrate <== Success \n");
	return ret;
}

int XU_IMG_Set_Mirror(int fd, unsigned char Mirror)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_IMG_Set_Mirror  ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= 0; 
	__u8 xu_selector= 0;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	if(chip_id == CHIP_RER9420)
	{
		xu_unit = XU_RERVISION_SYS_ID;
		xu_selector = XU_RERVISION_SYS_IMG_SETTING;
		
		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x01;
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		xu_unit = XU_RERVISION_USR_ID;
		xu_selector = XU_RERVISION_USR_IMG_SETTING;

		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x01;
	}

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_IMG_Set_Mirror ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set data
	xu_data[0] = Mirror;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_IMG_Set_Mirror ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_IMG_Set_Mirror  0x%x <== Success \n",Mirror);

	return 0;
}

int XU_IMG_Get_Mirror(int fd, unsigned char *Mirror)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_IMG_Get_Mirror  ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= 0; 
	__u8 xu_selector= 0;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	if(chip_id == CHIP_RER9420)
	{
		xu_unit = XU_RERVISION_SYS_ID;
		xu_selector = XU_RERVISION_SYS_IMG_SETTING;
		
		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x01;
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		xu_unit = XU_RERVISION_USR_ID;
		xu_selector = XU_RERVISION_USR_IMG_SETTING;

		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x01;
	}

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_IMG_Get_Mirror ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get data
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0)
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_IMG_Get_Mirror ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	*Mirror = xu_data[0];

	TestAp_Printf(TESTAP_DBG_FLOW, "Mirror = %d\n",*Mirror);
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_IMG_Get_Mirror <== Success \n");
	
	return 0;
}

int XU_IMG_Set_Flip(int fd, unsigned char Flip)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_IMG_Set_Flip  ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= 0; 
	__u8 xu_selector= 0;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	if(chip_id == CHIP_RER9420)
	{
		xu_unit = XU_RERVISION_SYS_ID;
		xu_selector = XU_RERVISION_SYS_IMG_SETTING;
		
		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x02;
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		xu_unit = XU_RERVISION_USR_ID;
		xu_selector = XU_RERVISION_USR_IMG_SETTING;

		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x02;
	}

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_IMG_Set_Flip ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set data
	xu_data[0] = Flip;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_IMG_Set_Flip ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_IMG_Set_Flip  0x%x <== Success \n",Flip);

	return 0;
}

int XU_IMG_Get_Flip(int fd, unsigned char *Flip)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_IMG_Get_Flip  ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= 0; 
	__u8 xu_selector= 0;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	if(chip_id == CHIP_RER9420)
	{
		xu_unit = XU_RERVISION_SYS_ID;
		xu_selector = XU_RERVISION_SYS_IMG_SETTING;
		
		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x02;
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		xu_unit = XU_RERVISION_USR_ID;
		xu_selector = XU_RERVISION_USR_IMG_SETTING;

		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x02;
	}

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_IMG_Get_Flip ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get data
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0)
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_IMG_Get_Flip ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	*Flip = xu_data[0];

	TestAp_Printf(TESTAP_DBG_FLOW, "Flip = %d\n",*Flip);
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_IMG_Get_Flip <== Success \n");
	
	return 0;
}

int XU_IMG_Set_Color(int fd, unsigned char Color)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_IMG_Set_Color  ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= 0; 
	__u8 xu_selector= 0;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	if(chip_id == CHIP_RER9420)
	{
		xu_unit = XU_RERVISION_SYS_ID;
		xu_selector = XU_RERVISION_SYS_IMG_SETTING;
		
		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x03;
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		xu_unit = XU_RERVISION_USR_ID;
		xu_selector = XU_RERVISION_USR_IMG_SETTING;

		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x03;
	}

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_IMG_Set_Color ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set data
	xu_data[0] = Color;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_IMG_Set_Color ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_IMG_Set_Color  0x%x <== Success \n",Color);

	return 0;
}

int XU_IMG_Get_Color(int fd, unsigned char *Color)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_IMG_Get_Color  ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= 0; 
	__u8 xu_selector= 0;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	if(chip_id == CHIP_RER9420)
	{
		xu_unit = XU_RERVISION_SYS_ID;
		xu_selector = XU_RERVISION_SYS_IMG_SETTING;
		
		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x03;
	}
	else if((chip_id == CHIP_RER9421)||(chip_id == CHIP_RER9422))
	{
		xu_unit = XU_RERVISION_USR_ID;
		xu_selector = XU_RERVISION_USR_IMG_SETTING;

		// Switch command
		xu_data[0] = 0x9A;				// Tag
		xu_data[1] = 0x03;
	}

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_IMG_Get_Color ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get data
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0)
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_IMG_Get_Color ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	*Color = xu_data[0];

	TestAp_Printf(TESTAP_DBG_FLOW, "Image Color = %d\n",*Color);
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_IMG_Get_Color <== Success \n");
	
	return 0;
}

//--------------------------------------------------------------------------------------
int XU_OSD_Set_CarcamCtrl(int fd, unsigned char SpeedEn, unsigned char CoordinateEn, unsigned char CoordinateCtrl)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_CarcamCtrl  ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x0A;
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_CarcamCtrl ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set data
	xu_data[0] = SpeedEn;
	xu_data[1] = CoordinateEn;
	xu_data[2] = CoordinateCtrl;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_CarcamCtrl ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_CarcamCtrl  0x%x  0x%x  0x%x <== Success \n", xu_data[0], xu_data[1], xu_data[2]);

	return 0;
}

int XU_OSD_Get_CarcamCtrl(int fd, unsigned char *SpeedEn, unsigned char *CoordinateEn, unsigned char *CoordinateCtrl)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_CarcamCtrl  ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x0A;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Get_CarcamCtrl ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get data
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0)
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Get_CarcamCtrl ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	*SpeedEn = xu_data[0];
	*CoordinateEn = xu_data[1];
	*CoordinateCtrl = xu_data[2];

	TestAp_Printf(TESTAP_DBG_FLOW, "OSD speed en = %d\n",*SpeedEn);
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD coordinate en = %d\n",*CoordinateEn);
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD coordinate ctrl = %d\n",*CoordinateCtrl);
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_CarcamCtrl <== Success \n");
	
	return 0;
}

int XU_OSD_Set_Speed(int fd, unsigned int Speed)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_Speed  ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x0B;
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_Speed ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set data
	xu_data[0] = (Speed >> 8) & 0xFF;
	xu_data[1] = Speed & 0xFF;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_Speed ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_Speed  0x%x  0x%x  <== Success \n", xu_data[0], xu_data[1]);

	return 0;
}

int XU_OSD_Get_Speed(int fd, unsigned int *Speed)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_Speed  ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x0B;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Get_Speed ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get data
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0)
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Get_Speed ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	*Speed = (xu_data[0]<<8) | (xu_data[1]);

	TestAp_Printf(TESTAP_DBG_FLOW, "OSD speed = %d \n",*Speed);
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_Speed <== Success \n");
	
	return 0;
}

int XU_OSD_Set_Coordinate1(int fd, unsigned char Direction, unsigned char *Vaule)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_Coordinate1  ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x0C;
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_Coordinate1 ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set data
	xu_data[0] = Direction;
	xu_data[1] = Vaule[0];
	xu_data[2] = Vaule[1];
	xu_data[3] = Vaule[2];
	xu_data[4] = 0;
	xu_data[5] = Vaule[3];
	xu_data[6] = Vaule[4];
	xu_data[7] = Vaule[5];
	xu_data[8] = 0;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_Coordinate1 ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_Coordinate  0x%x  0x%x  0x%x  0x%x  0x%x  0x%x  0x%x  0x%x 0x%x <== Success \n", xu_data[0], xu_data[1],
	xu_data[2], xu_data[3], xu_data[4], xu_data[5], xu_data[6], xu_data[7], xu_data[8]);

	return 0;
}

int XU_OSD_Set_Coordinate2(int fd, unsigned char Direction, unsigned char Vaule1, unsigned long Vaule2, unsigned char Vaule3, unsigned long Vaule4)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_Coordinate2  ==>\n");

	int err = 0;
	char i;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x0C;
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_Coordinate2 ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set data
	xu_data[0] = Direction;
	xu_data[1] = Vaule1;
	xu_data[2] = (Vaule2 >> 16) & 0xFF;
	xu_data[3] = (Vaule2 >> 8) & 0xFF;
	xu_data[4] = Vaule2 & 0xFF;
	
	xu_data[5] = Vaule3;
	xu_data[6] = (Vaule4 >> 16) & 0xFF;
	xu_data[7] = (Vaule4 >> 8) & 0xFF;
	xu_data[8] = Vaule4 & 0xFF;	

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Set_Coordinate2 ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Set_Coordinate2  0x%x  0x%x  0x%x  0x%x  0x%x  0x%x  0x%x  0x%x  0x%x <== Success \n", xu_data[0], xu_data[1],
	xu_data[2], xu_data[3], xu_data[4], xu_data[5], xu_data[6], xu_data[7], xu_data[8]);

	return 0;
}

int XU_OSD_Get_Coordinate1(int fd, unsigned char *Direction, unsigned char *Vaule)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_Coordinate1  ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x0C;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Get_Coordinate1 ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get data
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0)
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Get_Coordinate1 ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	*Direction = xu_data[0];
	Vaule[0] = xu_data[1];
	Vaule[1] = xu_data[2];
	Vaule[2] = xu_data[3];
	Vaule[3] = xu_data[5];
	Vaule[4] = xu_data[6];
	Vaule[5] = xu_data[7];
	
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Coordinate direction = %d\n",*Direction);
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Coordinate degree1 = %d\n",Vaule[0] );
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Coordinate minute1 = %d\n",Vaule[1] );
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Coordinate second1 = %d\n",Vaule[2] );
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Coordinate degree2 = %d\n",Vaule[3] );
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Coordinate minute2 = %d\n",Vaule[4] );
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Coordinate second2 = %d\n",Vaule[5] );
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_Coordinate1 <== Success \n");
	
	return 0;
}

int XU_OSD_Get_Coordinate2(int fd, unsigned char *Direction, unsigned char *Vaule1, unsigned long *Vaule2, unsigned char *Vaule3, unsigned long *Vaule4)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_Coordinate2  ==>\n");

	int err = 0;
	char i;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_OSD_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;				// Tag
	xu_data[1] = 0x0C;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Get_Coordinate2 ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get data
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0)
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_OSD_Get_Coordinate2 ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	*Direction = xu_data[0];
	*Vaule1 = xu_data[1];
	*Vaule2 =(xu_data[2]<<16) | (xu_data[3]<<8) | (xu_data[4]);
	*Vaule3 = xu_data[5];
	*Vaule4 =(xu_data[6]<<16) | (xu_data[7]<<8) | (xu_data[8]);
	
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Coordinate direction = %d\n",*Direction);
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Coordinate degree1 = %d.%05ld\n",*Vaule1, *Vaule2);
	TestAp_Printf(TESTAP_DBG_FLOW, "OSD Coordinate degree2 = %d.%05ld\n",*Vaule3, *Vaule4);
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_Coordinate2 <== Success \n");
	
	return 0;
}


int XU_GPIO_Ctrl_Set(int fd, unsigned char GPIO_En, unsigned char GPIO_Value)
{

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_GPIO_Ctrl_Set  ==>\n");

	int err = 0;
	char i;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_GPIO_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;						// Tag
	xu_data[1] = 0x01;
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_GPIO_Ctrl_Set ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set data
	xu_data[0] = GPIO_En;
	xu_data[1] = GPIO_Value;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_GPIO_Ctrl_Set ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_GPIO_Ctrl_Set  0x%x  0x%x <== Success \n", xu_data[0], xu_data[1]);

	return 0;

}



int XU_GPIO_Ctrl_Get(int fd, unsigned char *GPIO_En, unsigned char *GPIO_OutputValue, unsigned char *GPIO_InputValue)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_GPIO_Ctrl_Get  ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_GPIO_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;						// Tag
	xu_data[1] = 0x01;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_GPIO_Ctrl_Get ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get data
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0)
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_GPIO_Ctrl_Get ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	*GPIO_En = xu_data[0];
	*GPIO_OutputValue = xu_data[1];
    *GPIO_InputValue = xu_data[2];
	
	TestAp_Printf(TESTAP_DBG_FLOW, "GPIO enable = 0x%x\n",*GPIO_En);
	TestAp_Printf(TESTAP_DBG_FLOW, "GPIO Output value = 0x%x, Input value = 0x%x\n",*GPIO_OutputValue,*GPIO_InputValue);
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_GPIO_Ctrl_Get <== Success \n");
	
	return 0;
}



int XU_Frame_Drop_En_Set(int fd, unsigned char Stream1_En, unsigned char Stream2_En)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Frame_Drop_En_Set  ==>\n");

	int err = 0;
	char i;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_DYNAMIC_FPS_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;						// Tag
	xu_data[1] = 0x01;
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Frame_Drop_En_Set ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set data
	xu_data[0] = Stream1_En;
	xu_data[1] = Stream2_En;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Frame_Drop_En_Set ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Frame_Drop_En_Set  0x%x  0x%x <== Success \n", xu_data[0], xu_data[1]);

	return 0;
}

int XU_Frame_Drop_En_Get(int fd, unsigned char *Stream1_En, unsigned char *Stream2_En)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Frame_Drop_En_Get  ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_DYNAMIC_FPS_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;						// Tag
	xu_data[1] = 0x01;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Frame_Drop_En_Get ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get data
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0)
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Frame_Drop_En_Get ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	*Stream1_En = xu_data[0];
	*Stream2_En = xu_data[1];
	
	TestAp_Printf(TESTAP_DBG_FLOW, "Stream1 frame drop enable = %d\n",*Stream1_En);
	TestAp_Printf(TESTAP_DBG_FLOW, "Stream2 frame drop enable = %d\n",*Stream2_En);
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_Coordinate1 <== Success \n");
	
	return 0;
}

int XU_Frame_Drop_Ctrl_Set(int fd, unsigned char Stream1_fps, unsigned char Stream2_fps)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Frame_Drop_Ctrl_Set  ==>\n");

	int err = 0;
	char i;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_DYNAMIC_FPS_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;						// Tag
	xu_data[1] = 0x02;
	
	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Frame_Drop_Ctrl_Set ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Set data
	xu_data[0] = Stream1_fps;
	xu_data[1] = Stream2_fps;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Frame_Drop_Ctrl_Set ==> ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Frame_Drop_Ctrl_Set  0x%x  0x%x <== Success \n", xu_data[0], xu_data[1]);

	return 0;
}

int XU_Frame_Drop_Ctrl_Get(int fd, unsigned char *Stream1_fps, unsigned char *Stream2_fps)
{	
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_Frame_Drop_Ctrl_Get  ==>\n");

	int err = 0;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_USR_ID; 
	__u8 xu_selector= XU_RERVISION_USR_DYNAMIC_FPS_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	// Switch command
	xu_data[0] = 0x9A;						// Tag
	xu_data[1] = 0x02;

	if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Frame_Drop_Ctrl_Get ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}

	// Get data
	memset(xu_data, 0, xu_size);
	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0)
	{
		TestAp_Printf(TESTAP_DBG_ERR,"XU_Frame_Drop_Ctrl_Get ==> ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
		if(err==EINVAL)
			TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
		return err;
	}
	
	*Stream1_fps = xu_data[0];
	*Stream2_fps = xu_data[1];
	
	TestAp_Printf(TESTAP_DBG_FLOW, "Stream1 frame  = %d\n",*Stream1_fps);
	TestAp_Printf(TESTAP_DBG_FLOW, "Stream2 frame  = %d\n",*Stream2_fps);
	TestAp_Printf(TESTAP_DBG_FLOW, "XU_OSD_Get_Coordinate1 <== Success \n");
	
	return 0;
}


int XU_SF_Read(int fd, unsigned int Addr, unsigned char* pData,unsigned int Length)
{
    #define DEF_SF_RW_LENGTH 8
    #define min(a,b) a<b?a:b
	//TestAp_Printf(TESTAP_DBG_FLOW, "XU_SF_Read  ==>\n");

	int err = 0, i;
	unsigned int ValidLength = 0, loop = 0, remain = 0;
    unsigned char* pCopy = pData;
	__u8 ctrldata[11]={0};

	//uvc_xu_control parmeters
	__u8 xu_unit= XU_RERVISION_SYS_ID; 
	__u8 xu_selector= XU_RERVISION_SYS_FLASH_CTRL;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;


    if(Addr < 0x10000)
        ValidLength = min(0x10000 - Addr, Length);
    else
        ValidLength = min(0x20000 - Addr, Length);
    loop = ValidLength/DEF_SF_RW_LENGTH;
    remain = ValidLength%DEF_SF_RW_LENGTH;
    //TestAp_Printf(TESTAP_DBG_ERR,"valid = %d, loop = %d, remain = %d\n", ValidLength,loop,remain);
		
    //get sf data
    for(i = 0; i< loop; i++)
    {

        xu_data[0] = (Addr+i*DEF_SF_RW_LENGTH)&0xff;                     
        xu_data[1] = ((Addr+i*DEF_SF_RW_LENGTH)>>8)&0xff;
        if(Addr+i*DEF_SF_RW_LENGTH < 0x10000)
            xu_data[2] = 0x80 | DEF_SF_RW_LENGTH;
        else
            xu_data[2] = 0x90 | DEF_SF_RW_LENGTH;

        //set sf start addr
        if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
        {
            TestAp_Printf(TESTAP_DBG_ERR,"XU_SF_Read ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
            if(err==EINVAL)
                TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
            return err;
        }
        memset(xu_data, 0, xu_size*sizeof(__u8));
      	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
    	{
    		TestAp_Printf(TESTAP_DBG_ERR,"XU_SF_Read ==> Switch cmd : ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
    		if(err==EINVAL)
    			TestAp_Printf(TESTAP_DBG_ERR,"Read SF error\n");
    		return err;
    	}  
        memcpy(pCopy,xu_data+3,DEF_SF_RW_LENGTH);
        pCopy += DEF_SF_RW_LENGTH;
    }

    if(remain)
    {
        xu_data[0] = (Addr+loop*DEF_SF_RW_LENGTH)&0xff;                     
        xu_data[1] = ((Addr+loop*DEF_SF_RW_LENGTH)>>8)&0xff;
        if(Addr < 0x10000)
            xu_data[2] = 0x80 | remain;
        else
            xu_data[2] = 0x90 | remain;
        //set addr and length of remain 
        if ((err=XU_Set_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
        {
            TestAp_Printf(TESTAP_DBG_ERR,"XU_SF_Read ==> Switch cmd : ioctl(UVCIOC_CTRL_SET) FAILED (%i)\n",err);
            if(err==EINVAL)
                TestAp_Printf(TESTAP_DBG_ERR,"Invalid arguments\n");
            return err;
        }

        //get data of remain 
    
        memset(xu_data, 0, xu_size*sizeof(__u8));
      	if ((err=XU_Get_Cur(fd, xu_unit, xu_selector, xu_size, xu_data)) < 0) 
    	{
    		TestAp_Printf(TESTAP_DBG_ERR,"XU_SF_Read ==> Switch cmd : ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",err);
    		if(err==EINVAL)
    			TestAp_Printf(TESTAP_DBG_ERR,"Read SF error\n");
    		return err;
    	} 
        
        memcpy(pCopy,xu_data+3,remain);

    }
    //TestAp_Printf(TESTAP_DBG_FLOW, "XU_SF_Read <== Success \n");
    return 0;

}


