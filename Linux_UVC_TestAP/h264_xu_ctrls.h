#ifndef H264_XU_CTRLS_H
#define H264_XU_CTRLS_H

#include <linux/videodev2.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION (3, 0, 0)
#include <linux/uvcvideo.h>
#endif

#define CARCAM_PROJECT				0

/*
 * Dynamic controls
 */
//copy from uvcvideo.h
#define UVC_CTRL_DATA_TYPE_RAW		0
#define UVC_CTRL_DATA_TYPE_SIGNED	1
#define UVC_CTRL_DATA_TYPE_UNSIGNED	2
#define UVC_CTRL_DATA_TYPE_BOOLEAN	3
#define UVC_CTRL_DATA_TYPE_ENUM		4
#define UVC_CTRL_DATA_TYPE_BITMASK	5

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

#define V4L2_CID_BASE_EXTCTR_RERVISION				0x0A0c4501
#define V4L2_CID_BASE_RERVISION						V4L2_CID_BASE_EXTCTR_RERVISION
#define V4L2_CID_ASIC_RW_RERVISION					V4L2_CID_BASE_RERVISION+1
#define V4L2_CID_FLASH_CTRL					    	V4L2_CID_BASE_RERVISION+2
#define V4L2_CID_FRAME_INFO_RERVISION				V4L2_CID_BASE_RERVISION+3
#define V4L2_CID_H264_CTRL_RERVISION				V4L2_CID_BASE_RERVISION+4
#define V4L2_CID_MJPG_CTRL_RERVISION				V4L2_CID_BASE_RERVISION+5
#define V4L2_CID_OSD_CTRL_RERVISION					V4L2_CID_BASE_RERVISION+6
#define V4L2_CID_MOTION_DETECTION_RERVISION			V4L2_CID_BASE_RERVISION+7
#define V4L2_CID_IMG_SETTING_RERVISION				V4L2_CID_BASE_RERVISION+8
#define V4L2_CID_MULTI_STREAM_CTRL_RERVISION		V4L2_CID_BASE_RERVISION+9
#define V4L2_CID_GPIO_CTRL_RERVISION				V4L2_CID_BASE_RERVISION+10
#define V4L2_CID_DYNAMIC_FPS_CTRL_RERVISION			V4L2_CID_BASE_RERVISION+11
#define V4L2_CID_LAST_EXTCTR_RERVISION				V4L2_CID_DYNAMIC_FPS_CTRL_RERVISION

/* ---------------------------------------------------------------------------- */

#define UVC_GUID_RERVISION_SYS_HW_CTRL				{0x70, 0x33, 0xf0, 0x28, 0x11, 0x63, 0x2e, 0x4a, 0xba, 0x2c, 0x68, 0x90, 0xeb, 0x33, 0x40, 0x16}
#define UVC_GUID_RERVISION_USR_HW_CTRL				{0x94, 0x73, 0xDF, 0xDD, 0x3E, 0x97, 0x27, 0x47, 0xBE, 0xD9, 0x04, 0xED, 0x64, 0x26, 0xDC, 0x67}
//#define UVC_GUID_RERVISION_USR_HW_CTRL				{0x3F, 0xAE, 0x12, 0x28, 0xD7, 0xBC, 0x11, 0x4E, 0xA3, 0x57, 0x6F, 0x1E, 0xDE, 0xF7, 0xD6, 0x1D}

#define XU_RERVISION_SYS_ID							0x03
#define XU_RERVISION_USR_ID							0x04

// ----------------------------- XU Control Selector ------------------------------------
#define XU_RERVISION_SYS_ASIC_RW	      			   0x01
#define XU_RERVISION_SYS_FLASH_CTRL					0x03
#define XU_RERVISION_SYS_FRAME_INFO					0x06
#define XU_RERVISION_SYS_H264_CTRL					0x07
#define XU_RERVISION_SYS_MJPG_CTRL					0x08
#define XU_RERVISION_SYS_OSD_CTRL					0x09
#define XU_RERVISION_SYS_MOTION_DETECTION			0x0A
#define XU_RERVISION_SYS_IMG_SETTING				0x0B

#define XU_RERVISION_USR_FRAME_INFO					0x01
#define XU_RERVISION_USR_H264_CTRL					0x02
#define XU_RERVISION_USR_MJPG_CTRL					0x03
#define XU_RERVISION_USR_OSD_CTRL					0x04
#define XU_RERVISION_USR_MOTION_DETECTION			0x05
#define XU_RERVISION_USR_IMG_SETTING				0x06
#define XU_RERVISION_USR_MULTI_STREAM_CTRL			0x07
#define XU_RERVISION_USR_GPIO_CTRL					0x08
#define XU_RERVISION_USR_DYNAMIC_FPS_CTRL			0x09

// ----------------------------- XU Control Selector ------------------------------------
//copy from uvcvideo.h
#define UVC_CONTROL_SET_CUR	(1 << 0)
#define UVC_CONTROL_GET_CUR	(1 << 1)
#define UVC_CONTROL_GET_MIN	(1 << 2)
#define UVC_CONTROL_GET_MAX	(1 << 3)
#define UVC_CONTROL_GET_RES	(1 << 4)
#define UVC_CONTROL_GET_DEF	(1 << 5)
/* Control should be saved at suspend and restored at resume. */
#define UVC_CONTROL_RESTORE	(1 << 6)
/* Control can be updated by the camera. */
#define UVC_CONTROL_AUTO_UPDATE	(1 << 7)

#define UVC_CONTROL_GET_RANGE   (UVC_CONTROL_GET_CUR | UVC_CONTROL_GET_MIN | \
                                 UVC_CONTROL_GET_MAX | UVC_CONTROL_GET_RES | \
                                 UVC_CONTROL_GET_DEF)

struct uvc_xu_control_info {
	__u8 entity[16];
	__u8 index;
	__u8 selector;
	__u16 size;
	__u32 flags;
};

#if LINUX_VERSION_CODE <= KERNEL_VERSION (3, 0, 0)
struct uvc_xu_control_mapping {
	__u32 id;
	__u8 name[32];
	__u8 entity[16];
	__u8 selector;

	__u8 size;
	__u8 offset;
#if LINUX_VERSION_CODE == KERNEL_VERSION(3,0,0) || LINUX_VERSION_CODE == KERNEL_VERSION(3,0,35)
	__u32 v4l2_type;
	__u32 data_type;

	struct uvc_menu_info __user *menu_info;
	__u32 menu_count;

	__u32 reserved[4];
#else    
	enum v4l2_ctrl_type v4l2_type;
	__u32 data_type;
#endif    
};
#endif

struct uvc_xu_control {
	__u8 unit;
	__u8 selector;
	__u16 size;
	__u8 *data;
};

struct H264Format
{
	unsigned short  wWidth;
	unsigned short  wHeight;
	int   		fpsCnt;
	unsigned int	FrameSize;
	unsigned int	*FrPay;		// FrameInterval[0|1]Payloadsize[2|3]
};

struct Cur_H264Format
{	
	int FmtId;
	unsigned short wWidth;
	unsigned short wHeight;
	int FrameRateId;
	unsigned char framerate;
	unsigned int FrameSize;
};

struct Multistream_Info
{
	unsigned char strm_type;
	unsigned int format; 
};

typedef enum{
	CHIP_NONE = -1,
	CHIP_RER9420 = 0,
	CHIP_RER9421,
	CHIP_RER9422
}CHIP_RER942X;

#if LINUX_VERSION_CODE > KERNEL_VERSION (3, 0, 0)
#define UVC_SET_CUR					0x01
#define UVC_GET_CUR					0x81
#define UVCIOC_CTRL_MAP		_IOWR('u', 0x20, struct uvc_xu_control_mapping)
#define UVCIOC_CTRL_QUERY	_IOWR('u', 0x21, struct uvc_xu_control_query)
#else
#define UVCIOC_CTRL_ADD		_IOW('U', 1, struct uvc_xu_control_info)
#define UVCIOC_CTRL_MAP		_IOWR('U', 2, struct uvc_xu_control_mapping)
#define UVCIOC_CTRL_GET		_IOWR('U', 3, struct uvc_xu_control)
#define UVCIOC_CTRL_SET		_IOW('U', 4, struct uvc_xu_control)
#endif
extern unsigned int  chip_id;

int XU_Init_Ctrl(int fd);
int XU_Ctrl_ReadChipID(int fd);

int H264_GetFormat(int fd);
int H264_CountFormat(unsigned char *Data, int len);
int H264_ParseFormat(unsigned char *Data, int len, struct H264Format *fmt);
int H264_GetFPS(unsigned int FrPay);

// H.264 XU +++++

int XU_Set(int fd, struct uvc_xu_control xctrl);
int XU_Get(int fd, struct uvc_xu_control *xctrl);

int XU_Set_Cur(int fd, __u8 xu_unit, __u8 xu_selector, __u16 xu_size, __u8 *xu_data);
int XU_Get_Cur(int fd, __u8 xu_unit, __u8 xu_selector, __u16 xu_size, __u8 *xu_data);

int XU_H264_InitFormat(int fd);
int XU_H264_GetFormatLength(int fd, unsigned short *fwLen);
int XU_H264_GetFormatData(int fd, unsigned char *fwData, unsigned short fwLen);
int XU_H264_SetFormat(int fd, struct Cur_H264Format fmt);

int XU_H264_Get_QP_Limit(int fd, int *QP_Min, int *QP_Max);
int XU_H264_Get_Mode(int fd, int *Mode);
int XU_H264_Set_Mode(int fd, int Mode);

int XU_H264_Get_QP(int fd, int *QP_Val);
int XU_H264_Set_QP(int fd, int QP_Val);

int XU_H264_Get_BitRate(int fd, double *BitRate);
int XU_H264_Set_BitRate(int fd, double BitRate);

int XU_H264_Set_IFRAME(int fd);

int XU_H264_Get_SEI(int fd, unsigned char *SEI);
int XU_H264_Set_SEI(int fd, unsigned char SEI);

int XU_H264_Get_GOP(int fd, unsigned int *GOP);
int XU_H264_Set_GOP(int fd, unsigned int GOP);

int XU_Asic_Read(int fd, unsigned int Addr, unsigned char *AsicData);
int XU_Asic_Write(int fd, unsigned int Addr, unsigned char AsicData);

#if(CARCAM_PROJECT == 0)
int XU_Multi_Get_status(int fd, struct Multistream_Info *status);
int XU_Multi_Get_Info(int fd, struct Multistream_Info *Info);
int XU_Multi_Set_Type(int fd, unsigned int format);

int XU_Multi_Set_Enable(int fd, unsigned char enable);
int XU_Multi_Get_Enable(int fd, unsigned char *enable);

int XU_Multi_Set_BitRate(int fd, unsigned int StreamID, unsigned int BitRate);
int XU_Multi_Get_BitRate(int fd, unsigned int StreamID, unsigned int *BitRate);
int XU_Multi_Set_QP(int fd, unsigned int StreamID, unsigned int QP_Val);
int XU_Multi_Get_QP(int fd, unsigned int StreamID, unsigned int *QP_val);
int XU_Multi_Set_H264Mode(int fd, unsigned int StreamID, unsigned int Mode);
int XU_Multi_Get_H264Mode(int fd, unsigned int StreamID, unsigned int *Mode);
int XU_Multi_Set_SubStream_FrameRate(int fd, unsigned int sub_fps);
int XU_Multi_Get_SubStream_FrameRate(int fd, unsigned int *sub_fps);
int XU_Multi_Set_SubStream_GOP(int fd, unsigned int sub_gop);
int XU_Multi_Get_SubStream_GOP(int fd, unsigned int *sub_gop);
#endif

int XU_OSD_Timer_Ctrl(int fd, unsigned char enable);
int XU_OSD_Set_RTC(int fd, unsigned int year, unsigned char month, unsigned char day, unsigned char hour, unsigned char minute, unsigned char second);
int XU_OSD_Get_RTC(int fd, unsigned int *year, unsigned char *month, unsigned char *day, unsigned char *hour, unsigned char *minute, unsigned char *second);

int XU_OSD_Set_Size(int fd, unsigned char LineSize, unsigned char BlockSize);
int XU_OSD_Get_Size(int fd, unsigned char *LineSize, unsigned char *BlockSize);

int XU_OSD_Set_Color(int fd, unsigned char FontColor, unsigned char BorderColor);
int XU_OSD_Get_Color(int fd, unsigned char *FontColor, unsigned char *BorderColor);

int XU_OSD_Set_Enable(int fd, unsigned char Enable_Line, unsigned char Enable_Block);
int XU_OSD_Get_Enable(int fd, unsigned char *Enable_Line, unsigned char *Enable_Block);

int XU_OSD_Set_AutoScale(int fd, unsigned char Enable_Line, unsigned char Enable_Block);
int XU_OSD_Get_AutoScale(int fd, unsigned char *Enable_Line, unsigned char *Enable_Block);

int XU_OSD_Set_Start_Position(int fd, unsigned char OSD_Type, unsigned int RowStart, unsigned int ColStart);
int XU_OSD_Get_Start_Position(int fd, unsigned int *LineRowStart, unsigned int *LineColStart, unsigned int *BlockRowStart, unsigned int *BlockColStart);

#if(CARCAM_PROJECT == 0)
int XU_OSD_Set_Multi_Size(int fd, unsigned char Stream0, unsigned char Stream1, unsigned char Stream2);
int XU_OSD_Get_Multi_Size(int fd, unsigned char *Stream0, unsigned char *Stream1, unsigned char *Stream2);

int XU_OSD_Set_MS_Start_Position(int fd, unsigned char StreamID, unsigned char RowStart, unsigned char ColStart);
int XU_OSD_Get_MS_Start_Position(int fd, unsigned char *S0_Row, unsigned char *S0_Col, unsigned char *S1_Row, unsigned char *S1_Col, unsigned char *S2_Row, unsigned char *S2_Col);

int XU_OSD_Set_String(int fd, unsigned char group, char *String);
int XU_OSD_Get_String(int fd, unsigned char group, char *String);
#endif

int XU_MD_Set_Mode(int fd, unsigned char Enable);
int XU_MD_Get_Mode(int fd, unsigned char *Enable);

int XU_MD_Set_Threshold(int fd, unsigned int MD_Threshold);
int XU_MD_Get_Threshold(int fd, unsigned int *MD_Threshold);

int XU_MD_Set_Mask(int fd, unsigned char *Mask);
int XU_MD_Get_Mask(int fd, unsigned char *Mask);

int XU_MD_Set_RESULT(int fd, unsigned char *Result);
int XU_MD_Get_RESULT(int fd, unsigned char *Result);

int XU_MJPG_Set_Bitrate(int fd, unsigned int MJPG_Bitrate);
int XU_MJPG_Get_Bitrate(int fd, unsigned int *MJPG_Bitrate);

int XU_IMG_Set_Mirror(int fd, unsigned char Mirror);
int XU_IMG_Get_Mirror(int fd, unsigned char *Mirror);

int XU_IMG_Set_Flip(int fd, unsigned char Flip);
int XU_IMG_Get_Flip(int fd, unsigned char *Flip);

int XU_IMG_Set_Color(int fd, unsigned char Color);
int XU_IMG_Get_Color(int fd, unsigned char *Color);
// H.264 XU -----

int XU_GPIO_Ctrl_Set(int fd, unsigned char GPIO_En, unsigned char GPIO_Value);
int XU_GPIO_Ctrl_Get(int fd, unsigned char *GPIO_En, unsigned char *GPIO_OutputValue, unsigned char *GPIO_InputValue);

int XU_Frame_Drop_En_Set(int fd, unsigned char Stream1_En, unsigned char Stream2_En);
int XU_Frame_Drop_En_Get(int fd, unsigned char *Stream1_En, unsigned char *Stream2_En);
int XU_Frame_Drop_Ctrl_Set(int fd, unsigned char Stream1_fps, unsigned char Stream2_fps);
int XU_Frame_Drop_Ctrl_Get(int fd, unsigned char *Stream1_fps, unsigned char *Stream2_fps);
int XU_SF_Read(int fd, unsigned int Addr, unsigned char* pData,unsigned int Length);


#if(CARCAM_PROJECT == 1)
int XU_OSD_Set_CarcamCtrl(int fd, unsigned char SpeedEn, unsigned char CoordinateEn, unsigned char CoordinateCtrl);
int XU_OSD_Get_CarcamCtrl(int fd, unsigned char *SpeedEn, unsigned char *CoordinateEn, unsigned char *CoordinateCtrl);
int XU_OSD_Set_Speed(int fd, unsigned int Speed);
int XU_OSD_Get_Speed(int fd, unsigned int *Speed);
int XU_OSD_Set_Coordinate1(int fd, unsigned char Direction, unsigned char *Vaule);
int XU_OSD_Set_Coordinate2(int fd, unsigned char Direction, unsigned char Vaule1, unsigned long Vaule2, unsigned char Vaule3, unsigned long Vaule4);
int XU_OSD_Get_Coordinate1(int fd, unsigned char *Direction, unsigned char *Vaule);
int XU_OSD_Get_Coordinate2(int fd, unsigned char *Direction, unsigned char *Vaule1, unsigned long *Vaule2, unsigned char *Vaule3, unsigned long *Vaule4);
#endif

#endif
