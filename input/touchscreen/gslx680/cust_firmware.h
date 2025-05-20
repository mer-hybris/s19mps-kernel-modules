#ifndef __CUST_FIRMWARE_H__
#define __CUST_FIRMWARE_H__

#if defined(CONFIG_PROJ_C64_HUAWF) /* GSL_DAC_ID */
    #define  DAC_IDENTIFY_PAGE_ADDRESS      {0x01,0xfe,0x10,0x00}
    #define  DAC_IDENTIFY_OFFSET            0x10
    #include "firmware/c64_huawf/gsl_chip_cfg1.h"
    #include "firmware/c64_huawf/gsl_chip_cfg2.h"
    #define  DAC_IDENTIFY_ID2_RULES         {{dac_index(0),"<",0xf}}
    #define  GSL_CONFIG_DATA_ID_CFG2        gsl_config_data_id_cfg2
    #define  GSLX680_FW_CFG2                gslx680_fw_cfg2

#elif defined(CONFIG_PROJ_E101_TENGZHI) /* GSL_CHIP_ID */
    #include "firmware/e101_tengzhi/gsl_chip_cfg1.h"
    #include "firmware/e101_tengzhi/gsl_chip_cfg2.h"
    #include "firmware/e101_tengzhi/gsl_tp_check_fw.h"
    #define GSL_C       100
    #define GSL_CHIP_1  0x00130010 //jiu
    #define GSL_CHIP_2  0x00138010 //xin
    #define GSL_CONFIG_DATA_ID_CFG2    gsl_config_data_id_cfg2
    #define GSLX680_FW_CFG2            gslx680_fw_cfg2

#elif defined(CONFIG_PROJ_C64_UCT)
    #include "firmware/c64_uct.h"

#elif defined(CONFIG_PROJ_B961_SUOMAI)
    #include "firmware/b961_suomai/b961_suomai.h"
    
#elif defined(CONFIG_PROJ_E64_UCT)
    #include "firmware/e64_uct.h"

#elif defined(CONFIG_PROJ_C64_FROG)
    #include "firmware/c64_testuct.h"

#elif defined(CONFIG_PROJ_S868_ZXT)
	#if defined(PRJ_FEATURE_H_GSLX680_VENDOR_ID_ZT1010)
    	#include "firmware/s868_zxt_zt1010.h"
	#else
    	#include "firmware/s868_zxt.h"
	#endif
	
#elif defined(CONFIG_PROJ_S30_ZXT)
    #ifdef CONFIG_PROJ_S30_ZXT_ZT1003T
	  #include "firmware/s30_zxt_zt1003t.h"
	#else
      #include "firmware/s30_zxt.h"
  #endif	
    
#elif defined(CONFIG_PROJ_T823_UCT)
	#ifdef CONFIG_PROJ_T823_UCT_1093
	  #include "firmware/t823_uct_1093.h"
	#else
    #include "firmware/t823_uct.h"
  #endif	
#elif defined(CONFIG_PROJ_G55_TAIYI)
    #include "firmware/gslx680_FW_c55_taiyi.h"

#elif defined(CONFIG_PROJ_C64_RUIOU)
    #include "firmware/gslx680_ruiou.h"

#elif defined(CONFIG_PROJ_B55_TAIYI_105A)
    #include "firmware/gslx680_b55_taiyi_105a.h"

#elif defined(CONFIG_PROJ_B823_UCT)
    #include "firmware/b823_uct.h"

#elif defined(CONFIG_PROJ_E62_CHUANQI)
    #include "firmware/E62_ChuanQi_GSL915.h"

#elif defined(CONFIG_PROJ_C88_UCT)
    #include "firmware/c88_uct/c88_uct.h"

#elif defined(CONFIG_PROJ_C801_TENGZHI)||defined(CONFIG_PROJ_E801_TENGZHI)\
    ||defined(CONFIG_PROJ_L801_TENGZHI)||defined(CONFIG_PROJ_B801_TENGZHI)\
    ||defined(CONFIG_PROJ_T801_TENGZHI)
    #include "firmware/c801_tengzhi.h"

#elif defined(CONFIG_PROJ_C801_UCT)
    #include "firmware/c801_uct.h"

#elif defined(CONFIG_PROJ_E823_UCT)
	#if defined(CONFIG_PROJ_E823_XX32_UCT)
		#include "firmware/e823_uct/e823_uct_xx32.h"
	#else
		#include "firmware/e823_uct/e823_uct.h"
	#endif

#elif defined(CONFIG_PROJ_E823_HUAYI)
	#if defined(CONFIG_PROJ_E823_HUAYI_ML1036P)
		#include "firmware/e823_huayi/e823_huayi_ml1036p.h"
	#else
		#include "firmware/e823_huayi/e823_huayi.h"
	#endif

#elif defined(CONFIG_PROJ_E823_REVO)
    #include "firmware/e823_revo/e823_revo.h"

#elif defined(CONFIG_PROJ_E823_SJWL)
    #include "firmware/e823_sjwl.h"
    
#elif defined(CONFIG_PROJ_C101_TENGZHI)
    #include "firmware/c101_tengzhi.h"

#elif defined(CONFIG_PROJ_L863_WEIHENG)
    #include "firmware/l863_weiheng.h"

#elif defined(CONFIG_PROJ_L30_HENGKE)
    #include "firmware/l30_hengke/l30_hengke.h"

#elif defined(CONFIG_PROJ_L30_GRTY)
    #include "firmware/l30_grty/l30_grty.h"

#elif defined(CONFIG_PROJ_L30_ZREN)
    #include "firmware/l30_zren/l30_zren.h"
	
#elif defined(CONFIG_PROJ_E863_SJWL)
    #if defined(CONFIG_PROJ_E863_SJWL_FHD)
        #include "firmware/e863_sjwl/e863_sjwl_fhd.h"
    #elif defined(PRJ_FEATURE_H_TOUCHSCREEN_GSLX680_FIRMWARE_GSL1680)
        #include "firmware/e863_sjwl/gsl1680_800x1280.h"
    #else
        #include "firmware/e863_sjwl/gsl3670_800x1280.h"
    #endif

#elif defined(CONFIG_PROJ_L863_SJWL)||defined(CONFIG_PROJ_C863_SJWL)||defined(CONFIG_PROJ_B863_SJWL)
    #include "firmware/x863_sjwl/l863_sjwl.h"

#elif defined(CONFIG_PROJ_C863_TENGZHI)
    #include "firmware/c863_tengzhi.h"

#elif defined(CONFIG_PROJ_B863_CHENGCHENG)
    #include "firmware/b863_chengcheng.h"

#elif defined(CONFIG_PROJ_B863_KENXINDA)
  #if defined(PRJ_FEATURE_H_BOARD_USE_NEW_TP)
    #include "firmware/b863_kenxinda/b863_kenxinda.h"
  #else
    #include "firmware/b863_kenxinda/b863_kenxinda_old.h"
  #endif
#elif defined(CONFIG_PROJ_L863_TENGZHI)||defined(CONFIG_PROJ_T863_TENGZHI)
    #include "firmware/l863_tengzhi.h"

#elif defined(CONFIG_PROJ_E863_CHENGCHENG)
    #include "firmware/e863_chengcheng.h"

#elif defined(CONFIG_PROJ_E863_YANGHUA)
    #include "firmware/e863_yanghua.h"

#elif defined(CONFIG_PROJ_E863_YIPUDA)
   #include "firmware/e863_yipuda.h"

#elif defined(CONFIG_PROJ_T864_SJWL)
    #include "firmware/t864_sjwl.h"
#elif defined(CONFIG_PROJ_T864_ZXT)
    #include "firmware/t864_zxt.h"    
#elif defined(CONFIG_PROJ_T864_SBYH)
    #include "firmware/t864_sbyh/t864_sbyh_800x1280.h"
#elif defined(CONFIG_PROJ_T864_KANGJIA)
    #include "firmware/t864_kangjia.h"
#elif defined(CONFIG_PROJ_T864_BALEI)
    #include "firmware/t864_balei/t864_balei_800x1280.h"
#elif defined(CONFIG_PROJ_L863_CHENGCHENG)
    #include "firmware/l863_fhd-chengcheng.h"

#elif defined(CONFIG_PROJ_E863_CHUANJY)
    #if defined(CONFIG_PROJ_E863_CHUANJY_ST081)
        #include "firmware/e863_chuanjy_st081.h"
    #else
        #include "firmware/e863_fhd-chuanjy.h"
    #endif

#elif defined(CONFIG_PROJ_L30_CHUANQI)
    #include "firmware/l30_chuanqi.h"

#elif defined(CONFIG_PROJ_B863_YANGHUA)
    #include "firmware/b863_yanghua.h"

#elif defined(CONFIG_PROJ_B863_QIPENG)
    #include "firmware/b863_qipeng.h"

#elif defined(CONFIG_PROJ_L863_QIPENG)
    #include "firmware/l863_qipeng_gsl3676_800x1280.h"

#elif defined(CONFIG_PROJ_B863_YIBOTONG)
    #include "firmware/b863_yibotong.h"

#elif defined(CONFIG_PROJ_B863_TENGXING)
    #include "firmware/b863_tengxing.h"
	
#elif defined(CONFIG_PROJ_B863_UCT)
    #include "firmware/b863_uct.h"

#elif defined(CONFIG_PROJ_C863_QIPENG)
    #include "firmware/c863_qipeng.h"

#elif defined(CONFIG_PROJ_E863_QIPENG)
    #include "firmware/e863_qipeng.h"

#elif defined(CONFIG_PROJ_E863_JDF)
    #include "firmware/e863_jdf.h"

#elif defined(CONFIG_PROJ_E863_SUOMAI)
    #include "firmware/e863_suomai.h"

#elif defined(CONFIG_PROJ_L864T_ZHUOXT)
    #include "firmware/l864t_zhuoxt.h"

#elif defined(CONFIG_PROJ_E863_AIKESHI)
    #include "firmware/e863_aikeshi.h"
#elif defined(CONFIG_PROJ_C863_JREN)
    #include "firmware/c863_jren.h"


#elif defined(CONFIG_PROJ_B863_JREN)
    #ifdef CONFIG_PROJ_B863_JREN_Z1063
    #include "firmware/b863_jren/b863_jren_800x1280.h"
    #elif defined(CONFIG_PROJ_B863_XX08_JREN_Z873)
    #include "firmware/b863_jren/b863_jren_z873.h"
    #elif defined(PRJ_FEATURE_H_BOARD_RESOLUTION_1024_600)
    #include "firmware/b863_jren/b863_jren_1024x600.h"
    #else
    #include "firmware/b863_jren/b863_jren.h"
    #endif

#elif defined(CONFIG_PROJ_C863_YANGHUA)
    #include "firmware/c863_yanghua.h"

#elif defined(CONFIG_PROJ_E863_CHUANQI)
    #include "firmware/e863_chuanqi.h"
#elif defined(CONFIG_PROJ_E863_ZHUOXT)
    #ifdef  CONFIG_PROJ_E863_ZHUOXT_ZT8002
    #include "firmware/e863_zhuoxt_zt8002.h"
    #else
    #include "firmware/e863_zhuoxt.h"
    #endif
#elif defined(CONFIG_PROJ_C863_TENGXING)
   #if defined(PRJ_FEATURE_H_BOARD_RESOLUTION_800_1280)
   #include "firmware/c863_tengxing_hd.h"
   #else
   #include "firmware/c863_tengxing.h"
   #endif

#elif defined(CONFIG_PROJ_E103_CHENGCHENG)
    #include "firmware/e103_chengcheng.h"

#elif defined(CONFIG_PROJ_E103_TENGZHI)
    #include "firmware/e103_tengzhi.h"

#elif defined(CONFIG_PROJ_E103_FROG)
    #include "firmware/e103_frog.h"

#elif defined(CONFIG_PROJ_E91_CHUANQI)
    #include "firmware/e91_chuanqi.h"

#elif defined(CONFIG_PROJ_E961_SUOMAI)
    #include "firmware/e961_suomai.h"

#elif defined(CONFIG_PROJ_E30_SUOMAI)
    #include "firmware/e30_suomai.h"

#elif defined(CONFIG_PROJ_E30_WEIDU)
    #include "firmware/e30_weidu.h"

#elif defined(CONFIG_PROJ_E30_WEIHENG)
    #include "firmware/e30_weiheng.h"

#elif defined(CONFIG_PROJ_E30_TENGXING)
    #include "firmware/e30_tengxing.h"

#elif defined(CONFIG_PROJ_E960_TENGZHI)
    #include "firmware/e960_tengzhi.h"

#elif defined(CONFIG_PROJ_E960_CHENGCHENG)
    #if defined(CONFIG_PROJ_E960_CHENGCHENG_FHD)
    #include "firmware/e960_chengcheng_fhd.h"
    #else
    #include "firmware/e960_chengcheng.h"
    #endif

#elif defined(CONFIG_PROJ_E960_MEIDIFEI)
    #include "firmware/e960_meidifei.h"

#elif defined(CONFIG_PROJ_E960_TENGXING)
    #include "firmware/e960_tengxing.h"

#elif defined(CONFIG_PROJ_E960_REVO)
    #include "firmware/e960_revo.h"

#elif defined(CONFIG_PROJ_L104_CHENGCHENG)
    #include "firmware/l104_chengcheng.h"

#elif defined(CONFIG_PROJ_L863_CHANGCHENG)
    #include "firmware/l863_changcheng.h"

#elif defined(CONFIG_PROJ_T30_ZXT)
    #include "firmware/t30_zxt.h"

#elif defined(CONFIG_PROJ_L863_CHUANGWEI)
    #include "firmware/l863_chuangwei/gsl3670.h"

#elif defined(CONFIG_PROJ_E960_MENGBO)
    #include "firmware/e960_mengbo.h"
	
#elif defined(CONFIG_PROJ_E961_CHUANGWEI)
    #include "firmware/e961_chuangwei.h"

#elif defined(CONFIG_PROJ_C960_ZHITENG)
    #include "firmware/c960_zhiteng.h"

#elif defined(CONFIG_PROJ_L30_KANGJIA)
    #include "firmware/l30_kangjia.h"

#elif defined(CONFIG_PROJ_L30A_TENGXING)
    #include "firmware/l30a_tengxing/l30a_tengxing.h"

#elif defined(CONFIG_PROJ_T864_JUNXIN)
    #include "firmware/t864_junxin.h"
#elif defined(CONFIG_PROJ_T864_YIBOTONG)
	#include "firmware/t864_yibotong.h"
#elif defined(CONFIG_PROJ_T864_JREN)
    #include "firmware/t864_jren.h"
	
#elif defined(CONFIG_PROJ_T864_JIANGYUAN)
    #if defined(CONFIG_PROJ_T864_XX32_JIANGYUAN_10INCH)
        #include "firmware/t864_jiangyuan_in10.h"
    #else
        #include "firmware/t864_jiangyuan.h"
    #endif 

#elif defined(CONFIG_PROJ_T864_YIPUDA)
    #include "firmware/t864_yipuda.h"

#elif defined(CONFIG_PROJ_T866_SJWL)
    #include "firmware/t866_sjwl.h"

#elif defined(CONFIG_PROJ_T30_AIHUA)
    #include "firmware/t30_aihua.h"

#elif defined(CONFIG_PROJ_T30_HUAYI)
    #include "firmware/t30_huayi/t30_huayi.h"
    #include "firmware/t30_huayi/t30_huayi_800_1280.h"

#elif defined(CONFIG_PROJ_T30_REVO)
	#if defined(CONFIG_PROJ_T30_REVO_HD)
		#include "firmware/t30_revo_hd.h"
	#else
		#include "firmware/t30_revo.h"
	#endif

#elif defined(CONFIG_PROJ_T30_MINGZHI)
    #include "firmware/t30_mingzhi/t30_mingzhi_800_1280.h"	

#elif defined(CONFIG_PROJ_B863_XINLKJ)
    #include "firmware/b863_xinlkj.h"

#elif defined(CONFIG_PROJ_S30_KDGK)
    #include "firmware/s30_kdgk.h"

#elif defined(CONFIG_PROJ_B863_AIHUA)
    #ifdef PRJ_FEATURE_H_BOARD_RESOLUTION_1024_600
	#include "firmware/b863_aihua/gsl1680_600x1024.h"
    #else
    #include "firmware/b863_aihua/gsl3670_800x1280.h"
    #endif

#elif defined(CONFIG_PROJ_L801_JREN) || defined(CONFIG_PROJ_E801_JREN)
    #include "firmware/l801_jren.h"

#elif defined(CONFIG_PROJ_E960_KDGK)
    #include "firmware/e960_kdgk.h"

#elif defined(CONFIG_PROJ_E960_GRTY)	
	#include "firmware/e960_grty/e960_grty.h"

#elif defined(CONFIG_PROJ_B64_ZHUOXT_540X1200)
    #include "firmware/b64_zhuoxt_540x1200.h"
#elif defined(CONFIG_PROJ_B706_SUOMAI)
    #include "firmware/b706_suomai.h"

#elif defined(CONFIG_PROJ_E30_ZXT) || defined(CONFIG_PROJ_L30A_ZHUOXT)
    #ifdef PRJ_FEATURE_H_BOARD_RESOLUTION_1200_1920
        #include "firmware/e30_zxt/e30_zxt_1200_1920.h"
    #else  
        #include "firmware/e30_zxt/e30_zxt_800_1280.h"
    #endif
#elif defined(CONFIG_PROJ_L30T_YIPUDA)
	#if defined(PRJ_FEATURE_H_BOARD_RESOLUTION_1200_1920) 
		#include "firmware/t30_yipuda.h"//fhd
	#else
		#include "firmware/t30_yipuda_hd.h"//hd
	#endif
#elif defined(CONFIG_PROJ_L865T_SJWL)
	#include "firmware/l865t_sjwl.h"
#elif defined(CONFIG_PROJ_L866T_TENGXING)
	#include "firmware/l866t_tengxing.h"
#elif defined(CONFIG_PROJ_L30_KDGK)
	#include "firmware/l30_kdgk.h"
#elif defined(CONFIG_PROJ_E88_UCT)
	#include "firmware/e88_uct.h"
#else
    #include "firmware/gslx680_ts.h"
#endif

#endif //__CUST_FIRMWARE_H__
