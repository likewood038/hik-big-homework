#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>

#include "fh_system_mpi.h"
#include "fh_vpu_mpi.h"
#include "fh_venc_mpi.h"
#include "FHAdv_OSD_mpi.h"
#include "fh_vb_mpipara.h"
#include "fh_vb_mpi.h"

#include "libdmc.h"
#include "libdmc_pes.h"
#include "libdmc_record_raw.h"
#include "font_array.h"
#include "sensor.h"

#include "vicap/fh_vicap_mpi.h"
#include <time.h>

/*123456123456*/
#define CHECK_RET(state, error_code)																	\
	if (state)																						\
	{																								\
		printf("[%s]:%d line [%s] return 0x%x ERROR\n",__FILE__,__LINE__ , __func__, error_code);	\
		return error_code;																			\
	}

#define ALIGN_UP(addr, edge)   ((addr + edge - 1) & ~(edge - 1))
#define ALIGN_BACK(addr, edge) ((edge) * (((addr) / (edge))))
#define ISP_PROC "/proc/driver/isp"
#define VPU_PROC "/proc/driver/vpu"
#define BGM_PROC "/proc/driver/bgm"
#define ENC_PROC "/proc/driver/enc"
#define JPEG_PROC "/proc/driver/jpeg"
#define TRACE_PROC "/proc/driver/trace"

#define WR_PROC_DEV(device,cmd)  do{ \
    int _tmp_fd; \
    _tmp_fd = open(device, O_RDWR, 0); \
    if(_tmp_fd >= 0) { \
        write(_tmp_fd, cmd, sizeof(cmd)); \
        close(_tmp_fd); \
    } \
}while(0)


static int g_sig_stop = 0;
static void sample_vlcview_handle_sig(int signo)
{
    g_sig_stop = 1;
}


struct isp_sensor_if sensor_func;

#define ISP_W0	3864
#define ISP_H0	2192
#define ISP_W	3840
#define ISP_H	2160
#define ISP_F	30	


static int g_get_stream_stop = 0;
static int g_get_stream_running = 0;


static time_t g_stopwatch_start = 0;  /* 秒表起始时间戳 */
static time_t g_record_start_time = 0; /* 录制开始时间戳 */
static int    g_recording   = 0;   /* 录制状态: 0=空闲, 1=录制中 */
static int    g_record_done = 0;   /* 录制完成标记 */
static int    g_record_error = 0;  /* 录制错误码: 0=正常, 1=subscribe失败, 2=unsubscribe失败 */

static pthread_t g_thread_stream;   /* stream线程句柄，用于安全退出join */

#define GROUP_ID 0 

int start_isp()
{
	int ret;
	int vimod = 0;//0:online 1:offline
	int vomod = 1;//1:to vpu 2:to ddr

	ISP_MEM_INIT stMemInit = {0};
	ISP_VI_ATTR_S vi_attr = {0};

	ISP_PARAM_CONFIG stIsp_para_cfg;
	unsigned int param_addr, param_size;
	char *isp_param_buff;
	Sensor_Init_t initConf = {0};
	
	FH_VICAP_DEV_ATTR_S stViDev = {0};
	FH_VICAP_VI_ATTR_S stViAttr = {0};

	FILE *param_file;

		

	isp_sensor_reset();


	stMemInit.enOfflineWorkMode   = vimod;
	stMemInit.enIspOutMode		  = vomod;
	stMemInit.enIspOutFmt		  = 1;//422 8bit
	stMemInit.stPicConf.u32Width  = ISP_W;
	stMemInit.stPicConf.u32Height = ISP_H;
	ret = API_ISP_MemInit(0, &stMemInit);
	CHECK_RET(ret != 0, ret);


	vi_attr.u16InputHeight = ISP_H;
	vi_attr.u16InputWidth  = ISP_W;
	vi_attr.u16PicHeight   = ISP_H;
	vi_attr.u16PicWidth    = ISP_W;
	vi_attr.u16FrameRate = 30;
	vi_attr.enBayerType	= BAYER_GBRG;
	ret = API_ISP_SetViAttr(0, &vi_attr);
	CHECK_RET(ret != 0, ret);


	sensor_func.init					= sensor_init_imx415;
	sensor_func.set_sns_fmt 			= sensor_set_fmt_imx415;
	sensor_func.set_sns_reg 			= sensor_write_reg;
	sensor_func.get_sns_reg 			= sensor_read_reg;
	sensor_func.set_exposure_ratio		= sensor_set_exposure_ratio_imx415;
	sensor_func.get_exposure_ratio		= sensor_get_exposure_ratio_imx415;
	sensor_func.get_sensor_attribute	= sensor_get_attribute_imx415;
	sensor_func.set_flipmirror			= sensor_set_mirror_flip_imx415;
	sensor_func.get_sns_ae_default		= GetAEDefault;
	sensor_func.get_sns_ae_info 		= GetAEInfo;
	sensor_func.set_sns_gain			= SetGain;
	sensor_func.set_sns_intt			= SetIntt;
	ret = API_ISP_SensorRegCb(0, 0, &sensor_func);
	CHECK_RET(ret != 0, ret);
	

	ret = API_ISP_SensorInit(0, &initConf);
	CHECK_RET(ret != 0, ret);
	

	ret = API_ISP_Init(0);
	CHECK_RET(ret != 0, ret);
	

	stViDev.enWorkMode = vimod;
	stViDev.stSize.u16Width  = ISP_W;
	stViDev.stSize.u16Height = ISP_H;
	ret = FH_VICAP_InitViDev(0, &stViDev);
	CHECK_RET(ret != 0, ret);
	

	stViAttr.enWorkMode = vimod;
	stViAttr.stInSize.u16Width		= ISP_W0;
	stViAttr.stInSize.u16Height 		= ISP_H0;
	stViAttr.stCropSize.bCutEnable		= 1;
	stViAttr.stCropSize.stRect.u16Width 	= ISP_W;
	stViAttr.stCropSize.stRect.u16Height	= ISP_H;
	ret = FH_VICAP_SetViAttr(0, &stViAttr);
	CHECK_RET(ret != 0, ret);

	if(vimod == 1)
	{
		FH_BIND_INFO src,dst;
		src.obj_id = FH_OBJ_VICAP;
		src.dev_id = 0;
		src.chn_id = 0;
		dst.obj_id = FH_OBJ_ISP;
		dst.dev_id = 0;
		dst.chn_id = 0;
		FH_SYS_Bind(src,dst);
	}
	

	ret = API_ISP_GetBinAddr(0, &stIsp_para_cfg);
	param_size = stIsp_para_cfg.u32BinSize;
	CHECK_RET(ret != 0, ret);
	
	

	isp_param_buff = (char*)malloc(param_size);
	param_file = fopen(SENSOR_PARAM, "rb");
	if(NULL == param_file)
	{
		free(isp_param_buff);
		
		printf("open file failed!\n");
		return -1;
	}
	
	if(param_size != fread(isp_param_buff, 1, param_size, param_file))
	{
		free(isp_param_buff);
		fclose(param_file);
		
		printf("open file failed!\n");
		return -1;
	}
	ret = API_ISP_LoadIspParam(0, isp_param_buff);
	CHECK_RET(ret != 0, ret);
	free(isp_param_buff);
	fclose(param_file);


	ret = isp_server_run();
	CHECK_RET(ret != 0, ret);

	return 0;
}


int isp_set_param(int key, int param)
{
	int ret;

	printf("isp set key %d, val %d\n", key, param);

	switch(key)
	{
		case ISP_AE: //AE[0,1]
			ret = isp_set_ae(param);
			break;	
		case ISP_AWB: //AWB[0,1]
			ret = isp_set_awb(param);
			break;
		case ISP_COLOR: //[1,255]
			ret = isp_set_saturation(param);
			break;
		case ISP_BRIGHT: //[0,255]
			ret = isp_set_bright(param);
			break;
		case ISP_NR: //[0,1]
			ret = isp_set_nr(param);
			break;
		case ISP_MF: //[0,3]
			ret = isp_set_mirrorflip(param);
			break;
		default:
			printf("Error: not support the key %d\n", key);
			break;
	}
	CHECK_RET(ret != 0, ret);

	return ret;
}


int sample_dmc_init(FH_CHAR *dst_ip, FH_UINT32 port ,FH_SINT32 max_channel_no)
{
    dmc_init();

    if (dst_ip != NULL && *dst_ip != 0)
    {
        dmc_pes_subscribe(max_channel_no, dst_ip, port);
    }

    return 0;
}

FH_VOID *sample_common_get_stream_proc(FH_VOID *arg)
{
    FH_SINT32 ret, i;
    FH_SINT32 end_flag;
    FH_SINT32 subtype;
    FH_VENC_STREAM stream;
    FH_SINT32 *stop = (FH_SINT32 *)arg;

    while (*stop == 0)
    {
        WR_PROC_DEV(TRACE_PROC, "timing_GetStream_START");


        ret = FH_VENC_GetStream_Block(FH_STREAM_ALL & (~(FH_STREAM_JPEG)), &stream);
        WR_PROC_DEV(TRACE_PROC, "timing_EncBlkFinish_xxx");

        if (ret != 0)
        {
            printf("Error(%d - %x): FH_VENC_GetStream_Block(FH_STREAM_ALL & (~(FH_STREAM_JPEG))) failed!\n", ret, ret);
            continue;
        }
		

        if (stream.stmtype == FH_STREAM_H264)
        {
            subtype = stream.h264_stream.frame_type == FH_FRAME_I ? DMC_MEDIA_SUBTYPE_IFRAME : DMC_MEDIA_SUBTYPE_PFRAME;
            for (i = 0; i < stream.h264_stream.nalu_cnt; i++)
            {
            	end_flag = (i == (stream.h264_stream.nalu_cnt - 1)) ? 1 : 0;
                dmc_input(stream.chan,
			    		  DMC_MEDIA_TYPE_H264,
			    		  subtype,
			    		  stream.h264_stream.time_stamp,
		    		      stream.h264_stream.nalu[i].start,
		    		      stream.h264_stream.nalu[i].length,
		    		      end_flag);
            }
        }


        else if (stream.stmtype == FH_STREAM_H265)
        {
            subtype = stream.h265_stream.frame_type == FH_FRAME_I ? DMC_MEDIA_SUBTYPE_IFRAME : DMC_MEDIA_SUBTYPE_PFRAME;
            for (i = 0; i < stream.h265_stream.nalu_cnt; i++)
			{
            	end_flag = (i == (stream.h265_stream.nalu_cnt - 1)) ? 1 : 0;
                dmc_input(stream.chan,
			    		  DMC_MEDIA_TYPE_H265,
			    		  subtype,
			    		  stream.h265_stream.time_stamp,
		    		      stream.h265_stream.nalu[i].start,
		    		      stream.h265_stream.nalu[i].length,
		    		      end_flag);
			}
        }


        else if (stream.stmtype == FH_STREAM_MJPEG)
        {
            dmc_input(stream.chan,
                      DMC_MEDIA_TYPE_MJPEG,
                      0,
                      0,
                      stream.mjpeg_stream.start,
                      stream.mjpeg_stream.length,
                      1);
        }

		//FH_VENC_GetStream
        ret = FH_VENC_ReleaseStream(&stream);
        if(ret)
        {
            printf("Error(%d - %x): FH_VENC_ReleaseStream failed for chan(%d)!\n", ret, ret, stream.chan);
        }
        WR_PROC_DEV(TRACE_PROC, "timing_GetStream_END");
    }

    *stop = 0;
    return NULL;
}

static int sampe_set_venc_cfg(int chan, int enc_w, int enc_h)
{
	FH_VENC_CHN_CAP cfg_vencmem;

    cfg_vencmem.support_type       = FH_NORMAL_H264|FH_NORMAL_H265;
    cfg_vencmem.max_size.u32Width  = ISP_W;
    cfg_vencmem.max_size.u32Height = ISP_H;
	
    int ret = FH_VENC_CreateChn(chan, &cfg_vencmem);
	if (ret != 0)
	{
	   return ret;
	}
	
	FH_VENC_CHN_CONFIG cfg_param= {0};

	cfg_param.chn_attr.enc_type 					 = FH_NORMAL_H264;
	cfg_param.chn_attr.h264_attr.profile			 = H264_PROFILE_MAIN;
	cfg_param.chn_attr.h264_attr.i_frame_intterval	 = 50;
	cfg_param.chn_attr.h264_attr.size.u32Width		 = enc_w;
	cfg_param.chn_attr.h264_attr.size.u32Height 	 = enc_h;

	cfg_param.rc_attr.rc_type						  = FH_RC_H264_CBR;
	cfg_param.rc_attr.rc_type = FH_RC_H264_CBR;
	cfg_param.rc_attr.h264_cbr.bitrate = 1800000;
	cfg_param.rc_attr.h264_cbr.init_qp = 35;
	cfg_param.rc_attr.h264_cbr.FrameRate.frame_count = 25;
	cfg_param.rc_attr.h264_cbr.FrameRate.frame_time = 1;
	cfg_param.rc_attr.h264_cbr.maxrate_percent = 100;
	cfg_param.rc_attr.h264_cbr.IFrmMaxBits = 0;
	cfg_param.rc_attr.h264_cbr.IP_QPDelta = 3;
	cfg_param.rc_attr.h264_cbr.I_BitProp = 5;
	cfg_param.rc_attr.h264_cbr.P_BitProp = 1;
	cfg_param.rc_attr.h264_cbr.fluctuate_level = 6;
	
	// cfg_param.rc_attr.h264_vbr.bitrate				  = 2*1024*1024;
	// cfg_param.rc_attr.h264_vbr.init_qp				  = 35;
	// cfg_param.rc_attr.h264_vbr.ImaxQP				  = 42;
	// cfg_param.rc_attr.h264_vbr.IminQP				  = 28;
	// cfg_param.rc_attr.h264_vbr.PmaxQP				  = 42;
	// cfg_param.rc_attr.h264_vbr.PminQP				  = 28;
	// cfg_param.rc_attr.h264_vbr.FrameRate.frame_count   = 25;
	// cfg_param.rc_attr.h264_vbr.FrameRate.frame_time	  = 1;
	// cfg_param.rc_attr.h264_vbr.maxrate_percent		  = 200;
	// cfg_param.rc_attr.h264_vbr.IFrmMaxBits			  = 0;
	// cfg_param.rc_attr.h264_vbr.IP_QPDelta			  = 3;
	// cfg_param.rc_attr.h264_vbr.I_BitProp 			  = 5;
	// cfg_param.rc_attr.h264_vbr.P_BitProp 			  = 1;
	// cfg_param.rc_attr.h264_vbr.fluctuate_level		  = 0;

	return FH_VENC_SetChnAttr(chan, &cfg_param);
}

static int sampe_set_jpeg_cfg(int chan, int enc_w, int enc_h, int quality)
{
	sleep(1);
	
	static int jpeg_cnt = 0;
	static int jpeg_init = 0;
	int ret = 0;

	if(jpeg_init == 0)
	{
		FH_VENC_CHN_CAP cfg_vencmem;

		cfg_vencmem.support_type       = FH_JPEG;
		cfg_vencmem.max_size.u32Width  = ISP_W;
		cfg_vencmem.max_size.u32Height = ISP_H;

		ret = FH_VENC_CreateChn(chan, &cfg_vencmem);
		if (ret != 0)
		{
			return ret;
		}
		
		jpeg_init = 1;
	}
	FH_VENC_CHN_CONFIG cfg_param = {0};

	cfg_param.chn_attr.enc_type = FH_JPEG;
	cfg_param.chn_attr.jpeg_attr.encode_speed = 4;
	cfg_param.chn_attr.jpeg_attr.qp = quality;

	ret = FH_VENC_SetChnAttr(chan, &cfg_param);
	CHECK_RET(ret != 0, ret);

	FH_BIND_INFO src,dst;
	src.obj_id = FH_OBJ_VPU_VO;
    src.dev_id = 0;
    src.chn_id = 0;

    dst.obj_id = FH_OBJ_JPEG;
    dst.dev_id = 0;
    dst.chn_id = chan;
	
    ret = FH_SYS_Bind(src, dst);
	CHECK_RET(ret != 0, ret);

	FH_VENC_STREAM jpeg_stream;

	while(1)
	{
		ret = FH_VENC_GetStream_Timeout(FH_STREAM_JPEG, &jpeg_stream,1000);
		if(ret == 0)
		{
			break;
		}
	}
	
	if (jpeg_stream.stmtype == FH_STREAM_JPEG)
	{
		char jpeg_path[50]  = {0};
		snprintf(jpeg_path, sizeof(jpeg_path), "/home/jpeg_%d.jpg",jpeg_cnt);
		FILE *jpeg_file = fopen(jpeg_path,"w+");
		if(jpeg_file)
		{
			fwrite(jpeg_stream.jpeg_stream.start, sizeof(char), jpeg_stream.jpeg_stream.length, jpeg_file);
			fclose(jpeg_file);
			printf("get jpeg file %d\n",jpeg_stream.jpeg_stream.length);
		}
		jpeg_cnt++;
	}
	ret = FH_VENC_ReleaseStream(&jpeg_stream);
	CHECK_RET(ret != 0, ret);

	ret = FH_SYS_UnBindbyDst(dst);
	CHECK_RET(ret != 0, ret);

	return 0;
}

int sample_set_osd()
{
	int ret;
    int graph_ctrl = 0;
	
    graph_ctrl |= FHT_OSD_GRAPH_CTRL_TOSD_AFTER_VP;

    /* OSD */
    ret = FHAdv_Osd_Init(0,FHT_OSD_DEBUG_LEVEL_ERROR, graph_ctrl, 0, 0);
    if (ret != FH_SUCCESS)
    {
        printf("FHAdv_Osd_Init failed with %x\n", ret);
        return ret;
    }

	/*asc*/
    FHT_OSD_FontLib_t font_lib;

	font_lib.pLibData = asc16;
	font_lib.libSize  = sizeof(asc16);
	ret = FHAdv_Osd_LoadFontLib(FHEN_FONT_TYPE_ASC, &font_lib);
	if (ret != 0)
	{
	    printf("Error: Load ASC font lib failed, ret=%d\n", ret);
	    return ret;
	}

    /* gb2312 */
    font_lib.pLibData = gb2312;
	font_lib.libSize  = sizeof(gb2312);
	ret = FHAdv_Osd_LoadFontLib(FHEN_FONT_TYPE_CHINESE, &font_lib);
	if (ret != 0)
	{
	    printf("Error: Load CHINESE font lib failed, ret=%d\n", ret);
	    return ret;
	}

	FHT_OSD_CONFIG_t osd_cfg;
    FHT_OSD_Layer_Config_t  pOsdLayerInfo[4];
    FHT_OSD_TextLine_t text_line_cfg[4];
    FH_CHAR text_data[4][128]; /*it should be enough*/
    FH_SINT32 user_defined_time = 0;

	memset(&osd_cfg, 0, sizeof(osd_cfg));
    memset(&pOsdLayerInfo[0], 0, 4 * sizeof(FHT_OSD_Layer_Config_t));
    memset(&text_line_cfg[0], 0, 4 * sizeof(FHT_OSD_TextLine_t));
    memset(&text_data, 0, sizeof(text_data));

	/*  */
    osd_cfg.osdRotate        = 0;
    osd_cfg.pOsdLayerInfo = &pOsdLayerInfo[0];

    osd_cfg.nOsdLayerNum     = 1;

    pOsdLayerInfo[0].layerStartX = 0;
    pOsdLayerInfo[0].layerStartY = 0;
    /* pOsdLayerInfo[0].layerMaxWidth = 640; */ /*???????????????????????????????????????��????????????????��???????*/
    /* pOsdLayerInfo[0].layerMaxHeight = 480; */

    pOsdLayerInfo[0].osdSize     = 64;


    pOsdLayerInfo[0].normalColor.fAlpha = 255;
    pOsdLayerInfo[0].normalColor.fRed   = 255;
    pOsdLayerInfo[0].normalColor.fGreen = 255;
    pOsdLayerInfo[0].normalColor.fBlue  = 255;


    pOsdLayerInfo[0].invertColor.fAlpha = 255;
    pOsdLayerInfo[0].invertColor.fRed   = 0;
    pOsdLayerInfo[0].invertColor.fGreen = 0;
    pOsdLayerInfo[0].invertColor.fBlue  = 0;


    pOsdLayerInfo[0].edgeColor.fAlpha = 255;
    pOsdLayerInfo[0].edgeColor.fRed   = 0;
    pOsdLayerInfo[0].edgeColor.fGreen = 0;
    pOsdLayerInfo[0].edgeColor.fBlue  = 0;


    pOsdLayerInfo[0].bkgColor.fAlpha = 0;


    pOsdLayerInfo[0].edgePixel        = 1;


    pOsdLayerInfo[0].osdInvertEnable  = FH_OSD_INVERT_DISABLE; /*disable???????*/
    pOsdLayerInfo[0].osdInvertThreshold.high_level = 180;
    pOsdLayerInfo[0].osdInvertThreshold.low_level  = 160;
    pOsdLayerInfo[0].layerFlag = FH_OSD_LAYER_USE_TWO_BUF;
    pOsdLayerInfo[0].layerId = 0;
	
	ret = FHAdv_Osd_Ex_SetText(0, 0, &osd_cfg);
    if (ret != FH_SUCCESS)
    {
		printf("FHAdv_Osd_Ex_SetText failed with %d\n", ret);
        return ret;
    }
	text_line_cfg[0].textInfo = text_data[0];
	text_line_cfg[1].textInfo = text_data[1];
	text_line_cfg[2].textInfo = text_data[2];
	text_line_cfg[3].textInfo = text_data[3];
	FH_CHAR user_tag_data[] = {
        0xe4, 0x0d+0, /*FHT_OSD_USER1,*/
        0x0a,               
        0xe4, 0x01, /*FHT_OSD_YEAR4, */
        '-',
        0xe4, 0x03, /*FHT_OSD_MONTH2, */
        '-',
        0xe4, 0x04, /*FHT_OSD_DAY, */
        0x20,       
        0xe4, 0x07, /*FHT_OSD_HOUR24, */
        ':',
        0xe4, 0x09, /*FHT_OSD_MINUTE, */
        ':',
        0xe4, 0x0a, /*FHT_OSD_SECOND, */
        0,          /*null terminated string*/
    };
#if 1
 	sprintf(text_line_cfg[0].textInfo, "Camera Channel - %d", 0);
	text_line_cfg[0].textEnable    = 1;
    text_line_cfg[0].timeOsdEnable = 0;
    text_line_cfg[0].textLineWidth = (64/2) * 36;
    text_line_cfg[0].linePositionX = 320; 
    text_line_cfg[0].linePositionY = 50;

    text_line_cfg[0].lineId = 0;
    text_line_cfg[0].enable = 1;

	ret = FHAdv_Osd_SetTextLine(0, 0, pOsdLayerInfo[0].layerId, &text_line_cfg[0]);
	if (ret != FH_SUCCESS)
	{
		printf("FHAdv_Osd_Ex_SetText failed with %d\n", ret);
		return ret;
	}
#endif	
#if 1
	strcat(text_line_cfg[1].textInfo, user_tag_data);
	text_line_cfg[1].textEnable    = 1;
    text_line_cfg[1].timeOsdEnable = 0;
    text_line_cfg[1].textLineWidth = (64/2) * 36;
    text_line_cfg[1].linePositionX = 640;
    text_line_cfg[1].linePositionY = 480;

    text_line_cfg[1].lineId = 1;
    text_line_cfg[1].enable = 1;

	ret = FHAdv_Osd_SetTextLine(0, 0, pOsdLayerInfo[0].layerId, &text_line_cfg[1]);
	if (ret != FH_SUCCESS)
	{
		printf("FHAdv_Osd_Ex_SetText failed with %d\n", ret);
		return ret;
	}
#endif

#if 1
    /* line 2: 码率显示（动态，每秒更新）- 初始值 */
    sprintf(text_line_cfg[2].textInfo, "Bitrate: 0.00 Mbps");
    text_line_cfg[2].textEnable    = 1;
    text_line_cfg[2].timeOsdEnable = 0;
    text_line_cfg[2].textLineWidth = (64 / 2) * 36;
    text_line_cfg[2].linePositionX = 50;
    text_line_cfg[2].linePositionY = 120;
    text_line_cfg[2].lineId        = 2;
    text_line_cfg[2].enable        = 1;

    ret = FHAdv_Osd_SetTextLine(0, 0, pOsdLayerInfo[0].layerId, &text_line_cfg[2]);
    if (ret != FH_SUCCESS)
    {
        printf("FHAdv_Osd_SetTextLine line2 failed with %d\n", ret);
        return ret;
    }
#endif

#if 1
    /* 初始化秒表计时器 */
    {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        g_stopwatch_start = now.tv_sec;
    }

    /* line 3: 秒表显示（动态，每秒更新）- 初始值 */
    sprintf(text_line_cfg[3].textInfo, "00:00:00");
    printf("[STOPWATCH] started at 00:00:00\n");

    text_line_cfg[3].textEnable    = 1;
    text_line_cfg[3].timeOsdEnable = 0;
    text_line_cfg[3].textLineWidth = (64 / 2) * 36;
    text_line_cfg[3].linePositionX = 640;
    text_line_cfg[3].linePositionY = 120;
    text_line_cfg[3].lineId        = 3;
    text_line_cfg[3].enable        = 1;

    ret = FHAdv_Osd_SetTextLine(0, 0, pOsdLayerInfo[0].layerId, &text_line_cfg[3]);
    if (ret != FH_SUCCESS)
    {
        printf("FHAdv_Osd_SetTextLine line3 failed with %d\n", ret);
        return ret;
    }
#endif
	return 0;
}

FH_VOID sample_common_media_driver_config(FH_VOID)
{
	VB_CONF_S stVbConf;
    FH_SINT32 ret;

    FH_VB_Exit();

    memset(&stVbConf, 0, sizeof(VB_CONF_S));
    stVbConf.u32MaxPoolCnt = 4;
    stVbConf.astCommPool[0].u32BlkSize = 3840 * 2160 * 3;
    stVbConf.astCommPool[0].u32BlkCnt = 4;
    stVbConf.astCommPool[1].u32BlkSize = 1920 * 1080 * 3;
    stVbConf.astCommPool[1].u32BlkCnt = 4;
    stVbConf.astCommPool[2].u32BlkSize = 1280 * 720 * 3;
    stVbConf.astCommPool[2].u32BlkCnt = 4;
    stVbConf.astCommPool[3].u32BlkSize = 768 * 448 * 3;
    stVbConf.astCommPool[3].u32BlkCnt = 4;

    ret = FH_VB_SetConf(&stVbConf);
    if (ret)
    {
        printf("[FH_VB_SetConf] failed with:%x\n", ret);
    }

    ret = FH_VB_Init();
    if (ret)
    {
        printf("[FH_VB_Init] failed with:%x\n", ret);
    }
	
	WR_PROC_DEV(ENC_PROC, "allchnstm_0_20000000_40");
    WR_PROC_DEV(ENC_PROC, "stm_20000000_40");
    WR_PROC_DEV(JPEG_PROC, "frmsize_1_3000000_3000000");
    WR_PROC_DEV(JPEG_PROC, "jpgstm_12000000_2");
    WR_PROC_DEV(JPEG_PROC, "mjpgstm_12000000_2");
	
}
/**
 * sample_record_state_machine
 *
 * 独立的录制状态机，基于 CLOCK_MONOTONIC 计时器。
 * 由主循环每秒调用一次。
 *
 * 状态流转:
 *   IDLE  --(首次调用)--> RECORDING --(60秒后)--> DONE
 *
 * 返回值: 0=正常, -1=subscribe失败, -2=unsubscribe失败
 */
static int sample_record_state_machine(void)
{
    struct timespec now_ts;
    int ret;

    /* 录制周期已完成，不再处理 */
    if (g_record_done)
        return 0;

    clock_gettime(CLOCK_MONOTONIC, &now_ts);

    /* ========== 状态1: 空闲 → 启动录制 ========== */
    if (!g_recording)
    {
        /* 确保 DMC record 输出目录 */
        chdir("/home");

        ret = dmc_record_subscribe(1);
        if (ret != 0)
        {
            printf("[RECORD] ERROR: dmc_record_subscribe(1) FAILED, "
                   "ret=0x%x (%d)\n", ret, ret);
            printf("[RECORD] HINT: check if DMC is initialized, "
                   "disk is writable, or /home exists\n");
            g_record_error = 1;
            /* 不设置 g_recording=1，下次循环自动重试 */
            return -1;
        }

        /* subscribe 成功 */
        g_recording         = 1;
        g_record_start_time = now_ts.tv_sec;
        g_record_error      = 0;
        printf("[RECORD] OK: dmc_record_subscribe(1) succeeded, "
               "file: /home/chan_0.h264, start_ts=%ld\n",
               (long)g_record_start_time);
        return 0;
    }

    /* ========== 状态2: 录制中 → 检查是否到达停止时间 ========== */
    if (now_ts.tv_sec - g_record_start_time >= 60)
    {
        ret = dmc_record_unsubscribe();
        if (ret != 0)
        {
            printf("[RECORD] ERROR: dmc_record_unsubscribe() FAILED, "
                   "ret=0x%x (%d)\n", ret, ret);
            printf("[RECORD] WARN: file may be incomplete or corrupted\n");
            g_record_error = 2;
            /* 即使 unsubscribe 失败也标记 done，避免死循环重试 */
        }
        else
        {
            printf("[RECORD] OK: dmc_record_unsubscribe() succeeded, "
                   "60s recording complete, file: /home/chan_0.h264\n");
        }

        g_recording = 0;
        g_record_done = 1;
        return (ret != 0) ? -2 : 0;
    }

    return 0;
}

static int sample_update_bitrate_osd(void)
{
    FH_CHN_STATUS status = {0};
    FHT_OSD_TextLine_t line_cfg = {0};
    FH_CHAR text[128] = {0};
    int ret;

    ret = FH_VENC_GetChnStatus(0, &status);
    if (ret != FH_SUCCESS)
    {
        printf("FH_VENC_GetChnStatus failed: 0x%x\n", ret);
        return ret;
    }

    snprintf(text, sizeof(text),
             "Bitrate: %.2f Mbps",
             status.bps / 1000000.0);

    line_cfg.textInfo = text;
    line_cfg.textEnable = 1;
    line_cfg.timeOsdEnable = 0;
    line_cfg.textLineWidth = (64 / 2) * 36;
    line_cfg.linePositionX = 50;
    line_cfg.linePositionY = 120;
    line_cfg.lineId = 2;
    line_cfg.enable = 1;

    ret = FHAdv_Osd_SetTextLine(0, 0, 0, &line_cfg);
    if (ret != FH_SUCCESS)
    {
        printf("Update bitrate OSD failed: 0x%x\n", ret);
    }


	{
        FHT_OSD_TextLine_t sw_line;
        FH_CHAR sw_text[128];
        struct timespec now_ts;
        time_t elapsed;
        int hh, mm, ss;

        clock_gettime(CLOCK_MONOTONIC, &now_ts);
        elapsed = now_ts.tv_sec - g_stopwatch_start;

        hh = (int)(elapsed / 3600);
        mm = (int)((elapsed % 3600) / 60);
        ss = (int)(elapsed % 60);

        memset(&sw_line, 0, sizeof(sw_line));
        memset(sw_text, 0, sizeof(sw_text));

        snprintf(sw_text, sizeof(sw_text), "%02d:%02d:%02d", hh, mm, ss);

        sw_line.textInfo      = sw_text;
        sw_line.textEnable    = 1;
        sw_line.timeOsdEnable = 0;
        sw_line.textLineWidth = (64 / 2) * 36;
        sw_line.linePositionX = 640;
        sw_line.linePositionY = 120;
        sw_line.lineId        = 3;
        sw_line.enable        = 1;

        ret = FHAdv_Osd_SetTextLine(0, 0, 0, &sw_line);
        if (ret != FH_SUCCESS)
        {
            printf("Update stopwatch OSD failed: 0x%x\n", ret);
        }
    }


    /* ========== 录制状态机（独立函数） ========== */
    {
        int rec_ret = sample_record_state_machine();
        if (rec_ret != 0)
        {
            printf("[RECORD] state machine error: %d\n", rec_ret);
        }
    }

    return ret;
}
int main(int argc, char *argv[])
{
    int  ret;
    char *dst_ip;
    unsigned int port;


    signal(SIGINT,  sample_vlcview_handle_sig);
    signal(SIGQUIT, sample_vlcview_handle_sig);
    signal(SIGKILL, sample_vlcview_handle_sig);
    signal(SIGTERM, sample_vlcview_handle_sig);

    dst_ip = argc > 1 ? argv[1] : NULL;
    port   = argc > 2 ? strtol(argv[2], NULL, 0) : 1234;

	printf("demo_main driver_config\n");

	sample_common_media_driver_config();

	ret = FH_SYS_Init(); 
	CHECK_RET(ret != 0, ret);

	printf("start_isp\n");
	
	start_isp();
	printf("start_isp success\n");
	


	//VPU GROUP 
	FH_VPU_SET_GRP_INFO grp_info;
	grp_info.vi_max_size.u32Width = ISP_W;
	grp_info.vi_max_size.u32Height = ISP_H;
	grp_info.ycmean_en = 1;
	grp_info.ycmean_ds = 16;

    ret = FH_VPSS_CreateGrp(GROUP_ID, &grp_info);
	CHECK_RET(ret != 0, ret);

	
	FH_VPU_SIZE vi_pic;
	vi_pic.vi_size.u32Width  = ISP_W;
	vi_pic.vi_size.u32Height = ISP_H;
	vi_pic.crop_area.crop_en = 0;
	vi_pic.crop_area.vpu_crop_area.u32X = 0;
	vi_pic.crop_area.vpu_crop_area.u32Y = 0;
	vi_pic.crop_area.vpu_crop_area.u32Width = 0;
	vi_pic.crop_area.vpu_crop_area.u32Height = 0;

	ret = FH_VPSS_SetViAttr(GROUP_ID,&vi_pic);
	CHECK_RET(ret != 0, ret);

	ret = FH_VPSS_Enable(GROUP_ID, VPU_MODE_ISP);
	CHECK_RET(ret != 0, ret)

	FH_VPU_CHN_INFO chn_info = {0};
	chn_info.bgm_enable = 1;
	chn_info.cpy_enable = 1;
	chn_info.sad_enable = 1;
	chn_info.bgm_ds = 8;
	chn_info.chn_max_size.u32Width = ISP_W;
    chn_info.chn_max_size.u32Height = ISP_H;
	chn_info.out_mode = VPU_VOMODE_SCAN;
	chn_info.support_mode = 1<<chn_info.out_mode;
	chn_info.bufnum = 3;
	chn_info.max_stride = 0;
	ret = FH_VPSS_CreateChn(GROUP_ID, 0, &chn_info);
	CHECK_RET(ret != 0, ret);
	
	//chn
	// ret = FH_VPSS_CreateChn(GROUP_ID, 1, &chn_info);
	// CHECK_RET(ret != 0, ret);

    FH_VPU_CHN_CONFIG chn_attr;
    chn_attr.vpu_chn_size.u32Width  = 1280;
    chn_attr.vpu_chn_size.u32Height = 720;
	chn_attr.crop_area.crop_en = 0;
    chn_attr.crop_area.vpu_crop_area.u32X = 0;
    chn_attr.crop_area.vpu_crop_area.u32Y = 0;
    chn_attr.crop_area.vpu_crop_area.u32Width = 0;
    chn_attr.crop_area.vpu_crop_area.u32Height = 0;
    chn_attr.offset = 0;
    chn_attr.depth = 1;
    chn_attr.stride = 0;
    ret = FH_VPSS_SetChnAttr(GROUP_ID, 0, &chn_attr);
    CHECK_RET(ret != 0, ret);

	// FH_VPU_CHN_CONFIG chn_attr2;
    // chn_attr2.vpu_chn_size.u32Width  = 640;
    // chn_attr2.vpu_chn_size.u32Height = 360;
	// chn_attr2.crop_area.crop_en = 0;
    // chn_attr2.crop_area.vpu_crop_area.u32X = 0;
    // chn_attr2.crop_area.vpu_crop_area.u32Y = 0;
    // chn_attr2.crop_area.vpu_crop_area.u32Width = 0;
    // chn_attr2.crop_area.vpu_crop_area.u32Height = 0;
    // chn_attr2.offset = 0;
    // chn_attr2.depth = 1;
    // chn_attr2.stride = 0;
    // ret = FH_VPSS_SetChnAttr(GROUP_ID, 1, &chn_attr2);  //????VPU???????????????��
    // CHECK_RET(ret != 0, ret);



    ret = FH_VPSS_SetVOMode(GROUP_ID, 0, VPU_VOMODE_SCAN);
    CHECK_RET(ret != 0, ret);

	ret = FH_VPSS_OpenChn(GROUP_ID, 0);
	CHECK_RET(ret != 0, ret);

	ret = sampe_set_venc_cfg(0, 1280, 720);
	CHECK_RET(ret != 0, ret);


	// ret = FH_VPSS_SetVOMode(GROUP_ID, 1, VPU_VOMODE_SCAN);
    // CHECK_RET(ret != 0, ret);
	// ret = FH_VPSS_OpenChn(GROUP_ID, 1);
	// CHECK_RET(ret != 0, ret);
	// ret = sampe_set_venc_cfg(1, 640, 360);
	// CHECK_RET(ret != 0, ret);


	FH_BIND_INFO src, dst;
	src.obj_id = FH_OBJ_ISP;
    src.dev_id = 0;
    src.chn_id = 0;
    dst.obj_id = FH_OBJ_VPU_VI;
    dst.dev_id = 0;
    dst.chn_id = 0;
    ret = FH_SYS_Bind(src, dst);
	CHECK_RET(ret != 0, ret);
	
	ret = FH_VENC_StartRecvPic(0);
	CHECK_RET(ret != 0, ret);


	// ret = FH_VENC_StartRecvPic(1);
	// CHECK_RET(ret != 0, ret);
	
    src.obj_id = FH_OBJ_VPU_VO;
    src.dev_id = GROUP_ID;
    src.chn_id = 0;

    dst.obj_id = FH_OBJ_ENC;
    dst.dev_id = 0;
    dst.chn_id = 0;

    ret = FH_SYS_Bind(src, dst);
	CHECK_RET(ret != 0, ret);

	// src.obj_id = FH_OBJ_VPU_VO;
	// src.dev_id = GROUP_ID;
	// src.chn_id = 1;

	// dst.obj_id = FH_OBJ_ENC;
	// dst.dev_id = 0;
	// dst.chn_id = 1;

	// ret = FH_SYS_Bind(src, dst);
	// CHECK_RET(ret != 0, ret);
	// 
	sample_dmc_init(dst_ip, port, 1);
	
	pthread_attr_t attr;
	pthread_t thread_stream;
	
	if(!g_get_stream_running)
	{
		g_get_stream_running = 1;
		g_get_stream_stop = 0;
		pthread_attr_init(&attr);
		pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
		pthread_attr_setstacksize(&attr, 3 * 1024);
		pthread_create(&g_thread_stream, &attr, sample_common_get_stream_proc, &g_get_stream_stop);
	}
#if 1
	ret = sample_set_osd();
	CHECK_RET(ret != 0, ret);
#endif

	isp_set_param(ISP_AE, 1);  

	isp_set_param(ISP_AWB, 1); 

	isp_set_param(ISP_COLOR, 25);

	isp_set_param(ISP_BRIGHT, 125);

	isp_set_param(ISP_NR, 1);

	isp_set_param(ISP_MF, 3);
#if 1
	FH_VPU_MASK stVpumaskinfo;
	memset(&stVpumaskinfo,0x0,sizeof(FH_VPU_MASK));

	stVpumaskinfo.mask_enable[0] = 1;
	stVpumaskinfo.area_value[0].u32X = 0;
	stVpumaskinfo.area_value[0].u32Y = 0;
	stVpumaskinfo.area_value[0].u32Width = 200; 
	stVpumaskinfo.area_value[0].u32Height = 200; 

	stVpumaskinfo.color = 0xffda059a;// 0xff008080  0xffff8080  0xffda059a  0xff7abe29
	
	ret = FH_VPSS_SetMask(0, &stVpumaskinfo);
	CHECK_RET(ret != 0, ret);
#endif
#if 0
	sleep(2);
	ret = sampe_set_jpeg_cfg(1, 3840, 2160, 40);  //
	CHECK_RET(ret != 0, ret);

	ret = sampe_set_jpeg_cfg(1, 3840, 2160, 80);
	CHECK_RET(ret != 0, ret);
#endif
    while (!g_sig_stop)
    {
        sample_update_bitrate_osd();
		usleep(1000000);
    }

	printf("[EXIT] cleaning up...\n");

    /* 1. 停止本地录制（如果还在录） */
    if (g_recording)
    {
        dmc_record_unsubscribe();
        printf("[EXIT] record unsubscribed\n");
    }

    /* 2. 停止PES推流（关闭UDP socket + /home/h264.ps） */
    dmc_pes_unsubscribe();
    printf("[EXIT] PES unsubscribed\n");

    /* 3. 通知stream线程停止取帧 */
    g_get_stream_stop = 1;

    /* 4. 等待stream线程退出 */
    if (g_get_stream_running)
    {
        pthread_join(g_thread_stream, NULL);
        printf("[EXIT] stream thread joined\n");
    }

    /* 5. 清理DMC */
    dmc_deinit();
    printf("[EXIT] DMC deinitialized\n");
    return 0;
}

