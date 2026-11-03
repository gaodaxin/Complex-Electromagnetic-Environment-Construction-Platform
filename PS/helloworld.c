/******************************************************************************
*
* Copyright (C) 2009 - 2014 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/

/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include <stdarg.h>
#include "platform.h"
#include "xil_printf.h"
#include "xrfdc.h"
#include "xuartps.h"
#include "sleep.h"
#include <metal/log.h>
#include "metal/sys.h"

/* RF Data Converter 鐨勮澶� ID锛屾潵鑷� BSP 鐢熸垚鐨� xparameters.h銆� */
#define RFSOC_DIVICE    XPAR_XRFDC_0_DEVICE_ID

/* RFdc 椹卞姩閰嶇疆鎸囬拡鍜屽疄渚嬪璞★紝鍏ㄥ眬淇濆瓨锛屼緵鍒濆鍖栨祦绋嬩娇鐢ㄣ�� */
XRFdc_Config *ConfigPtr_RFSOC ;
XRFdc         InstancePtr_RFSOC ;
//XRFdc_DSA_Settings *SettingsPtr ;
//XRFdc_Cal_Freeze_Settings cal_set ;

XRFdc_Calibration_Coefficients coe_offset ;

/* 鏈枃浠跺唴鐨� RFdc 鍒濆鍖栧叆鍙ｅ拰鏈鏂板鐨� ADC Nyquist Zone 2 杈呭姪鍑芥暟銆� */
int rf_Init(void);
static int Rfdc_SetAllEnabledAdcNyquistZone2(XRFdc *Inst);
static int Rfdc_VerifyAllEnabledAdcNyquistZone2(XRFdc *Inst);
static int Rfdc_DisableAllEnabledAdcCalibrationOverrides(XRFdc *Inst);
static int Rfdc_ResetDacTile2AndCheckPll(XRFdc *Inst);
static int Rfdc_SetAllEnabledDacMixersToRealBypass(XRFdc *Inst);
void my_metal_default_log_handler(enum metal_log_level level, const char *format, ...);
int metal_log_init(void);
/* main() lives in main.c. This file only provides rf_Init(). */

/*****************************************************************************/
/**
*
* 鍦� RFdc CfgInitialize 鎴愬姛鍚庛�丄DC/DAC reset/startup 鍓嶏紝
* 閬嶅巻鎵�鏈夊惎鐢ㄧ殑 ADC tile/block锛屽苟鎶婃瘡涓� ADC block 璁剧疆涓虹浜屽濂庢柉鐗瑰尯銆�
*
* 鑳屾櫙锛�
* - ADC 杈撳叆棰戠巼鑼冨洿涓� 1.3 GHz ~ 2.3 GHz銆�
* - ADC 閲囨牱鐜囦负 2.4 GSPS銆�
* - 绗簩濂堝鏂壒鍖哄搴� Fs/2 ~ Fs锛屽嵆 1.2 GHz ~ 2.4 GHz銆�
* - RFdc API 涓� XRFDC_EVEN_NYQUIST_ZONE 绛変环浜� NyquistZone = 2銆�
*
* 娉ㄦ剰锛�
* - 涓嶅亣璁惧彧鍚敤 Tile0/Block0銆�
* - 蹇呴』閫氳繃 XRFdc_IsADCBlockEnabled() 鏌ヨ纭欢瀹為檯鍚敤鐨� ADC block銆�
* - 璁剧疆鍚庣珛鍗宠鍥烇紝闃叉 PS 绔厤缃拰 Vivado RFDC IP 閰嶇疆涓嶄竴鑷淬��
*
******************************************************************************/
static int Rfdc_SetAllEnabledAdcNyquistZone2(XRFdc *Inst)
{
    int enabled_count = 0;
    u32 tile;
    u32 block;
    u32 zone;
    u32 status;

    for (tile = 0U; tile < 4U; tile++) {
        for (block = 0U; block < 4U; block++) {
            /* 鍙鐞� RFdc 椹卞姩鎶ュ憡涓� enabled 鐨� ADC block銆� */
            if (XRFdc_IsADCBlockEnabled(Inst, tile, block) == 0U) {
                continue;
            }

            enabled_count++;

            /* 璁剧疆 ADC block 涓虹浜屽濂庢柉鐗瑰尯锛屽嵆 Even Nyquist Zone銆� */
            status = XRFdc_SetNyquistZone(Inst, XRFDC_ADC_TILE, tile, block,
                                          XRFDC_EVEN_NYQUIST_ZONE);
            if (status != XRFDC_SUCCESS) {
                printf("ERROR: XRFdc_SetNyquistZone failed for ADC Tile %u Block %u, status=%u\n",
                       tile, block, status);
                return XST_FAILURE;
            }

            /* 璁剧疆瀹屾垚鍚庣珛鍗宠鍥烇紝纭繚瀵勫瓨鍣ㄧ姸鎬佺‘瀹炰负 Zone 2銆� */
            status = XRFdc_GetNyquistZone(Inst, XRFDC_ADC_TILE, tile, block, &zone);
            if (status != XRFDC_SUCCESS) {
                printf("ERROR: XRFdc_GetNyquistZone failed for ADC Tile %u Block %u, status=%u\n",
                       tile, block, status);
                return XST_FAILURE;
            }

            /* 璇诲洖鍊煎繀椤讳弗鏍肩瓑浜� 2锛屽惁鍒欏仠姝㈠垵濮嬪寲锛岄伩鍏嶅悗缁噰鏍烽璋辫В閲婇敊璇�� */
            if (zone != XRFDC_EVEN_NYQUIST_ZONE) {
                printf("ERROR: ADC Tile %u Block %u Nyquist Zone readback = %u, expected 2\n",
                       tile, block, zone);
                return XST_FAILURE;
            }

            /* 涓插彛楠屾敹瑕佹眰锛氭瘡涓� enabled ADC block 閮借鎵撳嵃 Zone 2銆� */
            printf("ADC Tile %u Block %u Nyquist Zone = 2\n", tile, block);
        }
    }

    /* 濡傛灉涓�涓� ADC block 閮芥病妫�娴嬪埌锛岃鏄庣‖浠堕厤缃垨椹卞姩鐘舵�佸紓甯搞�� */
    if (enabled_count == 0) {
        printf("ERROR: No enabled ADC blocks detected\n");
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* ADC startup 瀹屾垚鍚庡啀娆¤鍥炴墍鏈� enabled ADC block 鐨� Nyquist Zone銆�
*
* 浣滅敤锛�
* - 纭 reset/startup 娴佺▼娌℃湁鎶� ADC Nyquist Zone 鎭㈠鎴栨敼鍐欍��
* - 涓婃澘涓插彛杈撳嚭姣忎釜 enabled ADC block 鐨勬渶缁堢姸鎬併��
* - 浠讳竴 block 璇诲洖涓嶆槸 Zone 2 鏃剁珛鍗冲け璐ヨ繑鍥烇紝閬垮厤 MTS 鍦ㄩ敊璇厤缃笅缁х画鎵ц銆�
*
******************************************************************************/
static int Rfdc_VerifyAllEnabledAdcNyquistZone2(XRFdc *Inst)
{
    int enabled_count = 0;
    u32 tile;
    u32 block;
    u32 zone;
    u32 status;

    for (tile = 0U; tile < 4U; tile++) {
        for (block = 0U; block < 4U; block++) {
            /* 浠嶇劧浠ラ┍鍔� API 鐨� enabled 鍒ゆ柇涓哄噯锛屼笉鎵嬪伐鐚滄祴鍚敤閫氶亾銆� */
            if (XRFdc_IsADCBlockEnabled(Inst, tile, block) == 0U) {
                continue;
            }

            enabled_count++;

            /* ADC startup 鍚庡彧璇诲洖楠岃瘉锛屼笉鍐嶉噸澶嶅啓鍏ラ厤缃�� */
            status = XRFdc_GetNyquistZone(Inst, XRFDC_ADC_TILE, tile, block, &zone);
            if (status != XRFDC_SUCCESS) {
                printf("ERROR: XRFdc_GetNyquistZone failed for ADC Tile %u Block %u, status=%u\n",
                       tile, block, status);
                return XST_FAILURE;
            }

            /* Zone 2 鏄湰宸ョ▼閲囬泦 1.3 GHz ~ 2.3 GHz 淇″彿鐨勫繀瑕侀厤缃�� */
            if (zone != XRFDC_EVEN_NYQUIST_ZONE) {
                printf("ERROR: ADC Tile %u Block %u Nyquist Zone readback = %u, expected 2\n",
                       tile, block, zone);
                return XST_FAILURE;
            }

            /* 淇濇寔鍜岃缃樁娈典竴鑷寸殑涓插彛鎵撳嵃锛屾柟渚块�� block 鏍稿銆� */
            printf("ADC Tile %u Block %u Nyquist Zone = 2\n", tile, block);
        }
    }

    /* 娌℃湁 enabled ADC block 鏃剁洿鎺ユ姤閿欙紝闃叉闈欓粯閫氳繃銆� */
    if (enabled_count == 0) {
        printf("ERROR: No enabled ADC blocks detected\n");
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

static int Rfdc_DisableAllEnabledAdcCalibrationOverrides(XRFdc *Inst)
{
	u32 tile;
	u32 block;
	u32 cal_block;
	const u32 cal_blocks[] = {
		XRFDC_CAL_BLOCK_OCB2,
		XRFDC_CAL_BLOCK_GCB,
		XRFDC_CAL_BLOCK_TSCB,
	};
	u32 status;

	for (tile = 0U; tile < 4U; tile++) {
		for (block = 0U; block < 4U; block++) {
			if (!XRFdc_IsADCBlockEnabled(Inst, tile, block)) {
				continue;
			}

			for (cal_block = 0U;
			     cal_block < (sizeof(cal_blocks) / sizeof(cal_blocks[0]));
			     cal_block++) {
				status = XRFdc_DisableCoefficientsOverride(
					Inst, tile, block, cal_blocks[cal_block]);
				if (status != XRFDC_SUCCESS) {
					printf("ERROR: disable ADC calibration override failed: "
					       "tile=%u block=%u cal=%u status=%u\\n",
					       tile, block, cal_blocks[cal_block], status);
					return XST_FAILURE;
				}
			}
		}
	}

	printf("INFO: ADC software calibration-coefficient override is disabled; automatic calibration remains enabled\\n");
	return XST_SUCCESS;
}

/* DAC tile 2 is the 300 MHz reference-clock source for the other DAC tiles. */
static int Rfdc_ResetDacTile2AndCheckPll(XRFdc *Inst)
{
	u32 reset_status;
	u32 pll_status = XRFDC_PLL_UNLOCKED;
	u32 pll_query_status;

	reset_status = XRFdc_Reset(Inst, XRFDC_DAC_TILE, 2U);
	pll_query_status = XRFdc_GetPLLLockStatus(
		Inst, XRFDC_DAC_TILE, 2U, &pll_status);
	printf("INFO: DAC Tile 2 reset=%u, internal PLL status=%s (0x%X)\\n",
	       reset_status,
	       (pll_status == XRFDC_PLL_LOCKED) ? "LOCKED" : "UNLOCKED",
	       pll_status);

	if (reset_status != XST_SUCCESS || pll_query_status != XRFDC_SUCCESS ||
	    pll_status != XRFDC_PLL_LOCKED) {
		printf("ERROR: DAC Tile 2 must lock before DAC tiles 0/1/3 are reset "
		       "(reset=%u pll_query=%u pll=0x%X)\\n",
		       reset_status, pll_query_status, pll_status);
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

/*
 * The DDS path supplies real, 16-sample-wide data directly to every enabled
 * DAC block.  Configure the corresponding RFDC data path as real-to-real
 * with the coarse mixer in BYPASS.  This is the supported RFDC representation
 * of a zero-Hz mixer for a real DAC path; MIXER_TYPE_OFF would instead select
 * MIXER_MODE_OFF and is not the R2R datapath required by this design.
 */
static int Rfdc_SetAllEnabledDacMixersToRealBypass(XRFdc *Inst)
{
	u32 tile;
	u32 block;
	u32 status;
	XRFdc_Mixer_Settings mixer;
	XRFdc_Mixer_Settings applied;

	for (tile = 0U; tile < 4U; tile++) {
		for (block = 0U; block < 4U; block++) {
			if (!XRFdc_IsDACBlockEnabled(Inst, tile, block)) {
				continue;
			}

			status = XRFdc_GetMixerSettings(Inst, XRFDC_DAC_TILE,
							     tile, block, &mixer);
			if (status != XRFDC_SUCCESS) {
				printf("ERROR: XRFdc_GetMixerSettings failed before bypass "
				       "for DAC Tile %u Block %u status=%u\\n",
				       tile, block, status);
				return XST_FAILURE;
			}
			printf("INFO: DAC Tile %u Block %u mixer before: type=%u mode=%u "
			       "coarse=%u freq=%.6f MHz phase=%.3f event=%u\\n",
			       tile, block, mixer.MixerType, mixer.MixerMode,
			       mixer.CoarseMixFreq, mixer.Freq, mixer.PhaseOffset,
			       mixer.EventSource);

			mixer.EventSource = XRFDC_EVNT_SRC_IMMEDIATE;
			mixer.Freq = 0.0;
			mixer.PhaseOffset = 0.0;
			mixer.CoarseMixFreq = XRFDC_COARSE_MIX_BYPASS;
			mixer.MixerMode = XRFDC_MIXER_MODE_R2R;
			mixer.FineMixerScale = XRFDC_MIXER_SCALE_AUTO;
			mixer.MixerType = XRFDC_MIXER_TYPE_COARSE;

			/* IMMEDIATE makes SetMixerSettings apply the mixer update now. */
			status = XRFdc_SetMixerSettings(Inst, XRFDC_DAC_TILE,
							     tile, block, &mixer);
			if (status != XRFDC_SUCCESS) {
				printf("ERROR: XRFdc_SetMixerSettings failed for DAC Tile %u "
				       "Block %u status=%u\\n", tile, block, status);
				return XST_FAILURE;
			}

			status = XRFdc_GetMixerSettings(Inst, XRFDC_DAC_TILE,
							     tile, block, &applied);
			if (status != XRFDC_SUCCESS) {
				printf("ERROR: XRFdc_GetMixerSettings failed after bypass "
				       "for DAC Tile %u Block %u status=%u\\n",
				       tile, block, status);
				return XST_FAILURE;
			}
			printf("INFO: DAC Tile %u Block %u mixer applied: type=%u mode=%u "
			       "coarse=%u freq=%.6f MHz phase=%.3f event=%u\\n",
			       tile, block, applied.MixerType, applied.MixerMode,
			       applied.CoarseMixFreq, applied.Freq, applied.PhaseOffset,
			       applied.EventSource);

			if ((applied.MixerType != XRFDC_MIXER_TYPE_COARSE) ||
			    (applied.MixerMode != XRFDC_MIXER_MODE_R2R) ||
			    (applied.CoarseMixFreq != XRFDC_COARSE_MIX_BYPASS) ||
			    (applied.Freq != 0.0) ||
			    (applied.PhaseOffset != 0.0) ||
			    (applied.EventSource != XRFDC_EVNT_SRC_IMMEDIATE)) {
				printf("ERROR: DAC Tile %u Block %u mixer readback is not "
				       "zero-Hz real bypass\\n", tile, block);
				return XST_FAILURE;
			}
		}
	}

	printf("INFO: All enabled DAC mixers are zero-Hz real bypass\\n");
	return XST_SUCCESS;
}

static int Rfdc_ConfigureEnabledDacZone2(XRFdc *Inst)
{
    u32 tile, block, zone = 0U, result;
    u16 invsinc = 0U;
    unsigned enabled = 0;

    for (tile = 0U; tile < 4U; tile++) {
        for (block = 0U; block < 4U; block++) {
            if (!XRFdc_IsDACBlockEnabled(Inst, tile, block))
                continue;
            enabled++;
            result = XRFdc_SetNyquistZone(Inst, XRFDC_DAC_TILE, tile,
                                         block, XRFDC_EVEN_NYQUIST_ZONE);
            if (result != XRFDC_SUCCESS) {
                printf("ERROR: DAC %u/%u SetNyquistZone status=%u\n",
                       tile, block, result);
                return XST_FAILURE;
            }
            result = XRFdc_GetNyquistZone(Inst, XRFDC_DAC_TILE, tile,
                                         block, &zone);
            if (result != XRFDC_SUCCESS || zone != XRFDC_EVEN_NYQUIST_ZONE) {
                printf("ERROR: DAC %u/%u Nyquist readback=%u status=%u\n",
                       tile, block, zone, result);
                return XST_FAILURE;
            }
            result = XRFdc_SetInvSincFIR(Inst, tile, block, 2U);
            if (result != XRFDC_SUCCESS) {
                printf("ERROR: DAC %u/%u SetInvSincFIR status=%u\n",
                       tile, block, result);
                return XST_FAILURE;
            }
            result = XRFdc_GetInvSincFIR(Inst, tile, block, &invsinc);
            if (result != XRFDC_SUCCESS || invsinc != 2U) {
                printf("ERROR: DAC %u/%u inverse-sinc readback=%u status=%u\n",
                       tile, block, (unsigned)invsinc, result);
                return XST_FAILURE;
            }
            printf("DAC Tile %u Block %u: Zone 2, inverse-sinc 2\n",
                   tile, block);
        }
    }
    if (enabled == 0U) {
        printf("ERROR: no enabled DAC blocks\n");
        return XST_FAILURE;
    }
    return XST_SUCCESS;
}

int rf_Init(void)
{
    int status ;
    int i ;
    XRFdc_MultiConverter_Sync_Config DAC_Sync_Config;
    XRFdc_MultiConverter_Sync_Config ADC_Sync_Config;

	/*
	 * 1. 鏌ユ壘 RFdc 閰嶇疆銆�
	 *    LookupConfig 杩斿洖 NULL 閫氬父璇存槑璁惧 ID 鎴� BSP 涓� RFdc 瀹炰緥閰嶇疆寮傚父銆�
	 */
	ConfigPtr_RFSOC = XRFdc_LookupConfig(RFSOC_DIVICE);
	if(ConfigPtr_RFSOC == NULL){
		printf("ERROR: XRFdc_LookupConfig failed\n");
		return XST_FAILURE;
	}

	/*
	 * 2. 鍒濆鍖� RFdc 椹卞姩瀹炰緥銆�
	 *    鍚庣画鎵�鏈� RFdc API 閮戒緷璧� InstancePtr_RFSOC 宸叉纭垵濮嬪寲銆�
	 */
    status = XRFdc_CfgInitialize(&InstancePtr_RFSOC,ConfigPtr_RFSOC);
	if(status != XST_SUCCESS){
		printf("ERROR: XRFdc_CfgInitialize failed, status=%d\n", status);
		return XST_FAILURE;
    }

	/*
	 * 3. 鍦� reset/startup 鍓嶈缃墍鏈� enabled ADC block 鐨� Nyquist Zone 2銆�
	 *    杩欐牱 PS 绔厤缃拰 Vivado RFDC IP 涓殑 Zone 2 璁剧疆淇濇寔涓�鑷淬��
	 */
	status = Rfdc_SetAllEnabledAdcNyquistZone2(&InstancePtr_RFSOC);
	if(status != XST_SUCCESS){
		return XST_FAILURE;
	}

	/* Do not apply OCB/GCB/TSCB values calibrated for the previous sample rate. */
	status = Rfdc_DisableAllEnabledAdcCalibrationOverrides(&InstancePtr_RFSOC);
	if(status != XST_SUCCESS){
		return XST_FAILURE;
	}
	//    衰    XRFdc_SetDSA
	/* 浠ヤ笅淇濈暀鍘熸湁 DSA/PLL/鏍″噯鐩稿叧璋冭瘯浠ｇ爜锛岄粯璁や笉鍚敤銆� */
//	  XRFdc_DSA_Settings.Attenuation = 10;
//	  XRFdc_SetDSA(&InstancePtr_RFSOC, 0, 0,  &SettingsPtr);
//	  XRFdc_SetDSA(&InstancePtr_RFSOC, 0, 1,  &SettingsPtr);
//	  XRFdc_SetDSA(&InstancePtr_RFSOC, 0, 2,  &SettingsPtr);
//	  XRFdc_SetDSA(&InstancePtr_RFSOC, 0, 3,  &SettingsPtr);
//	  XRFdc_SetDSA(&InstancePtr_RFSOC, 1, 0,  &SettingsPtr);
//	  XRFdc_SetDSA(&InstancePtr_RFSOC, 1, 1,  &SettingsPtr);
//	  XRFdc_SetDSA(&InstancePtr_RFSOC, 1, 2,  &SettingsPtr);
//	  XRFdc_SetDSA(&InstancePtr_RFSOC, 1, 3,  &SettingsPtr);
//	  XRFdc_SetDSA(&InstancePtr_RFSOC, 2, 0,  &SettingsPtr);
//	  XRFdc_SetDSA(&InstancePtr_RFSOC, 2, 1,  &SettingsPtr);
//	  XRFdc_SetDSA(&InstancePtr_RFSOC, 2, 2,  &SettingsPtr);
//	  XRFdc_SetDSA(&InstancePtr_RFSOC, 2, 3,  &SettingsPtr);
//	  XRFdc_SetDSA(&InstancePtr_RFSOC, 3, 0,  &SettingsPtr);
//	  XRFdc_SetDSA(&InstancePtr_RFSOC, 3, 1,  &SettingsPtr);
//	  XRFdc_SetDSA(&InstancePtr_RFSOC, 3, 2,  &SettingsPtr);
//	  XRFdc_SetDSA(&InstancePtr_RFSOC, 3, 3,  &SettingsPtr);
	/*u32 LockStatusPtr;

 for (i = 0 ; i < 4 ; i++){
	 status = XRFdc_GetPLLLockStatus(&InstancePtr_RFSOC,XRFDC_ADC_TILE, i,&LockStatusPtr);
	 status = XRFdc_GetPLLLockStatus(&InstancePtr_RFSOC,XRFDC_DAC_TILE, i,&LockStatusPtr);
 }
*/
	/*
	XRFdc_DisableCoefficientsOverride(&InstancePtr_RFSOC, 1, 0, XRFDC_CAL_BLOCK_OCB2);
	XRFdc_DisableCoefficientsOverride(&InstancePtr_RFSOC, 1, 0, XRFDC_CAL_BLOCK_GCB);
	XRFdc_DisableCoefficientsOverride(&InstancePtr_RFSOC, 1, 0, XRFDC_CAL_BLOCK_TSCB);


	XRFdc_DisableCoefficientsOverride(&InstancePtr_RFSOC, 1, 1, XRFDC_CAL_BLOCK_OCB2);
	XRFdc_DisableCoefficientsOverride(&InstancePtr_RFSOC, 1, 1, XRFDC_CAL_BLOCK_GCB);
	XRFdc_DisableCoefficientsOverride(&InstancePtr_RFSOC, 1, 1, XRFDC_CAL_BLOCK_TSCB);

	XRFdc_DisableCoefficientsOverride(&InstancePtr_RFSOC, 2, 0, XRFDC_CAL_BLOCK_OCB2);
	XRFdc_DisableCoefficientsOverride(&InstancePtr_RFSOC, 2, 0, XRFDC_CAL_BLOCK_GCB);
	XRFdc_DisableCoefficientsOverride(&InstancePtr_RFSOC, 2, 0, XRFDC_CAL_BLOCK_TSCB);

*/
	 usleep(10000);
/*
	// usleep(10000);
	 status = XRFdc_DynamicPLLConfig(&InstancePtr_RFSOC,XRFDC_DAC_TILE,0,XRFDC_INTERNAL_PLL_CLK ,250.0, 10000.0);
	 status = XRFdc_DynamicPLLConfig(&InstancePtr_RFSOC,XRFDC_DAC_TILE,1,XRFDC_INTERNAL_PLL_CLK ,250.0, 10000.0);
	 status = XRFdc_DynamicPLLConfig(&InstancePtr_RFSOC,XRFDC_DAC_TILE,2,XRFDC_INTERNAL_PLL_CLK ,250.0, 10000.0);
	 status = XRFdc_DynamicPLLConfig(&InstancePtr_RFSOC,XRFDC_DAC_TILE,3,XRFDC_INTERNAL_PLL_CLK ,250.0, 10000.0);
	//status = XRFdc_GetPLLLockStatus(&InstancePtr_RFSOC,XRFDC_DAC_TILE, 2,&LockStatusPtr);

*/



	/*
	 * 4. 鎸夊師鏈夋祦绋� reset ADC tile 0..3銆�
	 *    姣忔 reset 閮藉崟鐙鏌ヨ繑鍥炲�硷紝浠讳竴 tile 澶辫触閮藉仠姝㈠垵濮嬪寲銆�
	 */
	status = XRFdc_Reset(&InstancePtr_RFSOC,XRFDC_ADC_TILE,0);
	if(status != XST_SUCCESS){
		printf("ERROR: XRFdc_Reset failed for ADC Tile 0, status=%d\n", status);
		return XST_FAILURE;
	}
	status = XRFdc_Reset(&InstancePtr_RFSOC,XRFDC_ADC_TILE,1);
	if(status != XST_SUCCESS){
		printf("ERROR: XRFdc_Reset failed for ADC Tile 1, status=%d\n", status);
		return XST_FAILURE;
	}
	status = XRFdc_Reset(&InstancePtr_RFSOC,XRFDC_ADC_TILE,2);
	if(status != XST_SUCCESS){
		printf("ERROR: XRFdc_Reset failed for ADC Tile 2, status=%d\n", status);
		return XST_FAILURE;
	}
	status = XRFdc_Reset(&InstancePtr_RFSOC,XRFDC_ADC_TILE,3);
	if(status != XST_SUCCESS){
		printf("ERROR: XRFdc_Reset failed for ADC Tile 3, status=%d\n", status);
		return XST_FAILURE;
	}

	/*
	 * 5. DAC tile 2 uses the board 300 MHz reference and creates the shared
	 *    4.8 GSPS clock.  Bring it up first, verify its internal PLL, then
	 *    reset the dependent DAC tiles.
	 */
	status = Rfdc_ResetDacTile2AndCheckPll(&InstancePtr_RFSOC);
	if(status != XST_SUCCESS){
		return XST_FAILURE;
	}

    status = XRFdc_Reset(&InstancePtr_RFSOC,XRFDC_DAC_TILE,0);
	if(status != XST_SUCCESS){
		printf("ERROR: XRFdc_Reset failed for DAC Tile 0, status=%d\n", status);
		return XST_FAILURE;
	}
    status = XRFdc_Reset(&InstancePtr_RFSOC,XRFDC_DAC_TILE,1);
	if(status != XST_SUCCESS){
		printf("ERROR: XRFdc_Reset failed for DAC Tile 1, status=%d\n", status);
		return XST_FAILURE;
	}
	status = XRFdc_Reset(&InstancePtr_RFSOC,XRFDC_DAC_TILE,3);
	if(status != XST_SUCCESS){
		printf("ERROR: XRFdc_Reset failed for DAC Tile 3, status=%d\n", status);
		return XST_FAILURE;
	}


		usleep(10000);

	/*
	 * 6. 鍚姩鎵�鏈� ADC tile銆�
	 *    Tile 鍙傛暟 -1 鏄幇鏈夋祦绋嬶紝琛ㄧず鐢遍┍鍔ㄥ鐞嗗叏閮� ADC tile startup銆�
	 */
	status = XRFdc_StartUp(&InstancePtr_RFSOC, XRFDC_ADC_TILE,-1);
	 usleep(10000);
	if(status != XST_SUCCESS){
			printf("ERROR: XRFdc_StartUp failed for ADC tiles, status=%d\n", status);
			return XST_FAILURE;
	}

	/* ADC tile 2 is the board's 300 MHz reference-clock PLL source. */
	{
		u32 adc_pll_status = XRFDC_PLL_UNLOCKED;
		u32 query = XRFdc_GetPLLLockStatus(&InstancePtr_RFSOC,
			XRFDC_ADC_TILE, 2U, &adc_pll_status);
		printf("ADC Tile 2 PLL status=0x%x query=%u\n", adc_pll_status, query);
		if (query != XRFDC_SUCCESS || adc_pll_status != XRFDC_PLL_LOCKED) {
			printf("ERROR: ADC Tile 2 PLL is not locked\n");
			return XST_FAILURE;
		}
	}
	/*
	 * 7. ADC startup 鍚庡啀娆¤鍥� Nyquist Zone銆�
	 *    杩欐槸鏈�缁堣繘鍏� MTS 鍓嶇殑淇濇姢妫�鏌ャ��
	 */
	status = Rfdc_VerifyAllEnabledAdcNyquistZone2(&InstancePtr_RFSOC);
	if(status != XST_SUCCESS){
			return XST_FAILURE;
	}

	/*
	 * 8. Start the source tile before the dependent DAC tiles.
	 */
	status = XRFdc_StartUp(&InstancePtr_RFSOC, XRFDC_DAC_TILE,2);
	if(status != XST_SUCCESS){
		    printf("ERROR: XRFdc_StartUp failed for DAC Tile 2, status=%d\n", status);
		    return XST_FAILURE;
				}
	 status = XRFdc_StartUp(&InstancePtr_RFSOC, XRFDC_DAC_TILE,0);
	 if(status != XST_SUCCESS){
		    printf("ERROR: XRFdc_StartUp failed for DAC Tile 0, status=%d\n", status);
		    return XST_FAILURE;
   }

	status = XRFdc_StartUp(&InstancePtr_RFSOC, XRFDC_DAC_TILE,1);
		if(status != XST_SUCCESS){
		   printf("ERROR: XRFdc_StartUp failed for DAC Tile 1, status=%d\n", status);
		   return XST_FAILURE;
		}
	status = XRFdc_StartUp(&InstancePtr_RFSOC, XRFDC_DAC_TILE,3);
	if(status != XST_SUCCESS){
		printf("ERROR: XRFdc_StartUp failed for DAC Tile 3, status=%d\n", status);
		return XST_FAILURE;
	}

	/* Apply the analog Zone-2 output mode after DAC startup, then verify it. */
	status = Rfdc_ConfigureEnabledDacZone2(&InstancePtr_RFSOC);
	if (status != XST_SUCCESS)
		return XST_FAILURE;

	/* Override the stale coarse-mixer settings embedded in the RFDC IP. */
	status = Rfdc_SetAllEnabledDacMixersToRealBypass(&InstancePtr_RFSOC);
	if(status != XST_SUCCESS){
		return XST_FAILURE;
	}

	/* Keep DAC analog current aligned with working main.c (20 mA). */
	{
		u32 tile, block;
		u32 dac_output_current;
		XRFdc_DSA_Settings dsa = {0};
		dsa.DisableRTS = 1;
		dsa.Attenuation = 0;
		for (tile = 0U; tile < 4U; tile++) {
			for (block = 0U; block < 4U; block++) {
				if (XRFdc_IsDACBlockEnabled(&InstancePtr_RFSOC, tile, block)) {
					status = XRFdc_SetDACVOP(&InstancePtr_RFSOC, tile, block, 20000);
					if (status != XST_SUCCESS) {
						printf("ERROR: XRFdc_SetDACVOP failed Tile %u Block %u status=%d\n",
						       tile, block, status);
						return XST_FAILURE;
					}
					status = XRFdc_GetOutputCurr(&InstancePtr_RFSOC, tile,
								   block, &dac_output_current);
					if (status != XST_SUCCESS) {
						printf("ERROR: XRFdc_GetOutputCurr failed Tile %u Block %u status=%d\n",
						       tile, block, status);
						return XST_FAILURE;
					}
					printf("INFO: DAC Tile %u Block %u output current = %u uA\n",
					       tile, block, dac_output_current);
				}
				if (XRFdc_IsADCBlockEnabled(&InstancePtr_RFSOC, tile, block)) {
					status = XRFdc_SetDSA(&InstancePtr_RFSOC, tile, block, &dsa);
					if (status != XST_SUCCESS) {
						printf("ERROR: XRFdc_SetDSA failed Tile %u Block %u status=%d\n",
						       tile, block, status);
						return XST_FAILURE;
					}
				}
			}
		}
	}




		   /*
		    * 9. 鍒濆鍖� DAC/ADC MTS 閰嶇疆缁撴瀯銆�
		    *    淇濈暀鍘熸湁 tile mask锛�
		    *    - DAC_Sync_Config.Tiles = 0x3锛岃〃绀� DAC tile 0/1 鍙備笌 MTS銆�
		    *    - ADC_Sync_Config.Tiles = 0xF锛岃〃绀� ADC tile 0/1/2/3 鍙備笌 MTS銆�
		    */
		   status = XRFdc_MultiConverter_Init(&DAC_Sync_Config, 0, 0,0); // Initializer);
		   if(status != XRFDC_MTS_OK){
			   printf("ERROR: XRFdc_MultiConverter_Init failed for DAC MTS, status=%d\n", status);
			   return XST_FAILURE;
		   }
   	       status = XRFdc_MultiConverter_Init(&ADC_Sync_Config, 0, 0,0); // Initializer);
   	       if(status != XRFDC_MTS_OK){
   			   printf("ERROR: XRFdc_MultiConverter_Init failed for ADC MTS, status=%d\n", status);
   			   return XST_FAILURE;
   		   }
	   	   /* data_sel drives DAC tiles 0..3 (s00/s02 ... s30/s32). */
	   	   DAC_Sync_Config.Tiles = 0xF;
   		   ADC_Sync_Config.Tiles = 0xF;
   		   //ADC_Sync_Config.Target_Latency = -1;

	   /*
	    * 10. 鎵ц DAC Multi-Tile Sync銆�
	    *     浣跨敤 XRFDC_DAC_TILE 瀹忔浛浠ｈ８鏁板瓧 1锛岄伩鍏嶅悗缁淮鎶ゆ椂璇銆�
	    */
	   status = XRFdc_MultiConverter_Sync(&InstancePtr_RFSOC,XRFDC_DAC_TILE,&DAC_Sync_Config);
	   if(status == XRFDC_MTS_OK){
		   printf("INFO:DAC Multi-Tile-Sync completed successfully\n");
	   }else{
		   printf("INFO:DAC Multi-Tile-Sync completed error\n");
		   return XST_FAILURE;
	   }

	   /*
	    * 11. 鎵ц ADC Multi-Tile Sync銆�
	    *     浣跨敤 XRFDC_ADC_TILE 瀹忔浛浠ｈ８鏁板瓧 0锛屼繚鎸� RFdc API 璇箟娓呮櫚銆�
	    */
 	 	status = XRFdc_MultiConverter_Sync(&InstancePtr_RFSOC,XRFDC_ADC_TILE,&ADC_Sync_Config);
   	   	   if(status == XRFDC_MTS_OK){
	   		   printf("INFO:ADC Multi-Tile-Sync completed successfully\n");
	   	   }else{
	   		   printf("INFO:ADC Multi-Tile-Sync completed error\n");
	   		   return XST_FAILURE;
	   	   }

	   /* 鎵撳嵃鍙備笌 MTS 鐨� DAC tile latency锛屼繚鐣欏師鏈変覆鍙ｈ緭鍑恒�� */
	   for(i=0; i<4; i++) {
		   if((1<<i)&DAC_Sync_Config.Tiles) {
			   printf(" DAC%d: Latency(T1) =%3d\n", i,DAC_Sync_Config.Latency[i]);
	   	   }
	   }

	   /* 鎵撳嵃鍙備笌 MTS 鐨� ADC tile latency锛屼繚鐣欏師鏈変覆鍙ｈ緭鍑恒�� */
	   for(i=0; i<4; i++) {
	  		   if((1<<i)&ADC_Sync_Config.Tiles) {
	  			   printf(" ADCC%d: Latency(T1) =%3d\n", i,ADC_Sync_Config.Latency[i]);
	  	   	   }
	  	   }

	   /*
	    * 12. 鎸夊師鏈夋祦绋嬪叧闂� SYSREF 鎹曡幏/浼犺緭銆�
	    *     铏界劧褰撳墠 RFdc v11.0 瀹炵幇姝ｅ父杩斿洖 XRFDC_MTS_OK锛�
	    *     杩欓噷浠嶆鏌ヨ繑鍥炲�硷紝淇濊瘉鎵�鏈� RFdc API 澶辫触閮借兘琚覆鍙ｆ槑纭墦鍗般��
	    */
	   status = XRFdc_MTS_Sysref_Config(&InstancePtr_RFSOC,&DAC_Sync_Config,&ADC_Sync_Config, 0);
	   if(status != XRFDC_MTS_OK){
		   printf("ERROR: XRFdc_MTS_Sysref_Config failed, status=%d\n", status);
		   return XST_FAILURE;
	   }


/*
	   u32 j ;
	   for(i=0;i<4;i++){
		   for(j=0;j<4;j++){
			   XRFdc_GetCalCoefficients(&InstancePtr_RFSOC, i, j,XRFDC_CAL_BLOCK_TSCB , &coe_offset);
			   XRFdc_SetCalCoefficients(&InstancePtr_RFSOC, i, j,XRFDC_CAL_BLOCK_TSCB , &coe_offset);
			   XRFdc_GetCalCoefficients(&InstancePtr_RFSOC, i, j,XRFDC_CAL_BLOCK_TSCB , &coe_offset);

			   XRFdc_GetCalCoefficients(&InstancePtr_RFSOC, i, j,XRFDC_CAL_BLOCK_OCB2 , &coe_offset);
			   XRFdc_SetCalCoefficients(&InstancePtr_RFSOC, i, j,XRFDC_CAL_BLOCK_OCB2 , &coe_offset);

			   XRFdc_GetCalCoefficients(&InstancePtr_RFSOC, i, j,XRFDC_CAL_BLOCK_GCB , &coe_offset);
			   XRFdc_SetCalCoefficients(&InstancePtr_RFSOC, i, j,XRFDC_CAL_BLOCK_GCB , &coe_offset);


		   }
	   }
*/

	   /*
	    * These constants were measured with the old sampling-rate clock plan.
	    * Keep them in source for traceability only; applying them at 4.8 GSPS
	    * would override the RFDC's rate-appropriate automatic calibration.
	    */
#if 0
	   XRFdc_Calibration_Coefficients GCB_CoeffPtr[4][4];
	   XRFdc_Calibration_Coefficients TSCB_CoeffPtr[4][4];

//	   for(int i=0;i<4;i++){
//		 	for(int j=0;j<4;j=j+1)
//		 	{
//		 		status = XRFdc_GetCalCoefficients(&InstancePtr_RFSOC,i,j,XRFDC_CAL_BLOCK_GCB, &GCB_CoeffPtr[i][j]);
//		 		status = XRFdc_GetCalCoefficients(&InstancePtr_RFSOC,i,j,XRFDC_CAL_BLOCK_TSCB, &TSCB_CoeffPtr[i][j]);
//		 		//	status = XRFdc_GetCalCoefficients(&InstancePtr_RFSOC,i,j,XRFDC_CAL_BLOCK_OCB2, &OB2_CoeffPtr[i][j]);
//		 	}
//	   }

	   GCB_CoeffPtr[0][0].Coeff0 = 0x00000010;

	   	 GCB_CoeffPtr[0][0].Coeff1 = 0x00000fca;

	   	 GCB_CoeffPtr[0][0].Coeff2 = 0x00000feb;

	   	 GCB_CoeffPtr[0][0].Coeff3 = 0x00000fea;

	   	 GCB_CoeffPtr[0][0].Coeff4 = 0;

	   	 GCB_CoeffPtr[0][0].Coeff5 = 0;

	   	 GCB_CoeffPtr[0][0].Coeff6 = 0;

	   	 GCB_CoeffPtr[0][0].Coeff7 = 0;

	   	 GCB_CoeffPtr[0][1].Coeff0 = 0x00000013;

	   	 GCB_CoeffPtr[0][1].Coeff1 = 0x00000fc9;

	   	 GCB_CoeffPtr[0][1].Coeff2 = 0x00000fb6;

	   	 GCB_CoeffPtr[0][1].Coeff3 = 0x00000fec;

	   	 GCB_CoeffPtr[0][1].Coeff4 = 0;

	   	 GCB_CoeffPtr[0][1].Coeff5 = 0;

	   	 GCB_CoeffPtr[0][1].Coeff6 = 0;

	   	 GCB_CoeffPtr[0][1].Coeff7 = 0;
	   	GCB_CoeffPtr[0][2].Coeff0 = 0x00000ffa;

	   		   	 GCB_CoeffPtr[0][2].Coeff1 = 0x00000fef;

	   		   	 GCB_CoeffPtr[0][2].Coeff2 = 0x00000fe8;

	   		   	 GCB_CoeffPtr[0][2].Coeff3 = 0x00000fe5;

	   		   	 GCB_CoeffPtr[0][2].Coeff4 = 0;

	   		   	 GCB_CoeffPtr[0][2].Coeff5 = 0;

	   		   	 GCB_CoeffPtr[0][2].Coeff6 = 0;

	   		   	 GCB_CoeffPtr[0][2].Coeff7 = 0;

	   		   	 GCB_CoeffPtr[0][3].Coeff0 = 0x00000005;

	   		   	 GCB_CoeffPtr[0][3].Coeff1 = 0x00000031;

	   		   	 GCB_CoeffPtr[0][3].Coeff2 = 0x00000fef;

	   		   	 GCB_CoeffPtr[0][3].Coeff3 = 0x00000052;

	   		   	 GCB_CoeffPtr[0][3].Coeff4 = 0;

	   		   	 GCB_CoeffPtr[0][3].Coeff5 = 0;

	   		   	 GCB_CoeffPtr[0][3].Coeff6 = 0;

	   		   	 GCB_CoeffPtr[0][3].Coeff7 = 0;

	   	 GCB_CoeffPtr[1][0].Coeff0 = 0x0000000e;

	   	 GCB_CoeffPtr[1][0].Coeff1 = 0x00000033;

	   	 GCB_CoeffPtr[1][0].Coeff2 = 0x00000047;

	   	 GCB_CoeffPtr[1][0].Coeff3 = 0x00000032;

	   	 GCB_CoeffPtr[1][0].Coeff4 = 0;

	   	 GCB_CoeffPtr[1][0].Coeff5 = 0;

	   	 GCB_CoeffPtr[1][0].Coeff6 = 0;

	   	 GCB_CoeffPtr[1][0].Coeff7 = 0;

	   	 GCB_CoeffPtr[1][1].Coeff0 = 0x00000ff6;

	   	 GCB_CoeffPtr[1][1].Coeff1 = 0x0000000d;

	   	 GCB_CoeffPtr[1][1].Coeff2 = 0x00000ff2;

	   	 GCB_CoeffPtr[1][1].Coeff3 = 0x00000fbc;

	   	 GCB_CoeffPtr[1][1].Coeff4 = 0;

	   	 GCB_CoeffPtr[1][1].Coeff5 = 0;

	   	 GCB_CoeffPtr[1][1].Coeff6 = 0;

	   	 GCB_CoeffPtr[1][1].Coeff7 = 0;
	   	GCB_CoeffPtr[1][2].Coeff0 = 0x0000000d;

	   		   	 GCB_CoeffPtr[1][2].Coeff1 = 0x00000ff1;

	   		   	 GCB_CoeffPtr[1][2].Coeff2 = 0x0000002e;

	   		   	 GCB_CoeffPtr[1][2].Coeff3 = 0x00000fed;

	   		   	 GCB_CoeffPtr[1][2].Coeff4 = 0;

	   		   	 GCB_CoeffPtr[1][2].Coeff5 = 0;

	   		   	 GCB_CoeffPtr[1][2].Coeff6 = 0;

	   		   	 GCB_CoeffPtr[1][2].Coeff7 = 0;

	   		   	 GCB_CoeffPtr[1][3].Coeff0 = 0x00000ff4;

	   		   	 GCB_CoeffPtr[1][3].Coeff1 = 0x00000016;

	   		   	 GCB_CoeffPtr[1][3].Coeff2 = 0x00000027;

	   		   	 GCB_CoeffPtr[1][3].Coeff3 = 0x00000048;

	   		   	 GCB_CoeffPtr[1][3].Coeff4 = 0;

	   		   	 GCB_CoeffPtr[1][3].Coeff5 = 0;

	   		   	 GCB_CoeffPtr[1][3].Coeff6 = 0;

	   		   	 GCB_CoeffPtr[1][3].Coeff7 = 0;


	   	 GCB_CoeffPtr[2][0].Coeff0 = 0x00000010;

	   	 GCB_CoeffPtr[2][0].Coeff1 = 0x00000015;

	   	 GCB_CoeffPtr[2][0].Coeff2 = 0x00000fe2;

	   	 GCB_CoeffPtr[2][0].Coeff3 = 0x00000fed;

	   	 GCB_CoeffPtr[2][0].Coeff4 = 0;

	   	 GCB_CoeffPtr[2][0].Coeff5 = 0;

	   	 GCB_CoeffPtr[2][0].Coeff6 = 0;

	   	 GCB_CoeffPtr[2][0].Coeff7 = 0;

	   	 GCB_CoeffPtr[2][1].Coeff0 = 0x00000010;

	   	 GCB_CoeffPtr[2][1].Coeff1 = 0x00000fdf;

	   	 GCB_CoeffPtr[2][1].Coeff2 = 0x00000016;

	   	 GCB_CoeffPtr[2][1].Coeff3 = 0x00000fa9;

	   	 GCB_CoeffPtr[2][1].Coeff4 = 0;

	   	 GCB_CoeffPtr[2][1].Coeff5 = 0;

	   	 GCB_CoeffPtr[2][1].Coeff6 = 0;

	   	 GCB_CoeffPtr[2][1].Coeff7 = 0;

	   	GCB_CoeffPtr[2][2].Coeff0 = 0x00000ff5;

	   		   	 GCB_CoeffPtr[2][2].Coeff1 = 0x00000016;

	   		   	 GCB_CoeffPtr[2][2].Coeff2 = 0x0000000d;

	   		   	 GCB_CoeffPtr[2][2].Coeff3 = 0x00000ff1;

	   		   	 GCB_CoeffPtr[2][2].Coeff4 = 0;

	   		   	 GCB_CoeffPtr[2][2].Coeff5 = 0;

	   		   	 GCB_CoeffPtr[2][2].Coeff6 = 0;

	   		   	 GCB_CoeffPtr[2][2].Coeff7 = 0;

	   		   	 GCB_CoeffPtr[2][3].Coeff0 = 0x00000000;

	   		   	 GCB_CoeffPtr[2][3].Coeff1 = 0x00000002;

	   		   	 GCB_CoeffPtr[2][3].Coeff2 = 0x00000000;

	   		   	 GCB_CoeffPtr[2][3].Coeff3 = 0x00000002;

	   		   	 GCB_CoeffPtr[2][3].Coeff4 = 0;

	   		   	 GCB_CoeffPtr[2][3].Coeff5 = 0;

	   		   	 GCB_CoeffPtr[2][3].Coeff6 = 0;

	   		   	 GCB_CoeffPtr[2][3].Coeff7 = 0;

	   	 GCB_CoeffPtr[3][0].Coeff0 = 0x00000000;

	   	 GCB_CoeffPtr[3][0].Coeff1 = 0x0000000a;

	   	 GCB_CoeffPtr[3][0].Coeff2 = 0x00000009;

	   	 GCB_CoeffPtr[3][0].Coeff3 = 0x0000000d;

	   	 GCB_CoeffPtr[3][0].Coeff4 = 0;

	   	 GCB_CoeffPtr[3][0].Coeff5 = 0;

	   	 GCB_CoeffPtr[3][0].Coeff6 = 0;

	   	 GCB_CoeffPtr[3][0].Coeff7 = 0;

	   	 GCB_CoeffPtr[3][1].Coeff0 = 0x00000000;

	   	 GCB_CoeffPtr[3][1].Coeff1 = 0x00000ffa;

	   	 GCB_CoeffPtr[3][1].Coeff2 = 0x00000000;

	   	 GCB_CoeffPtr[3][1].Coeff3 = 0x00000fff;

	   	 GCB_CoeffPtr[3][1].Coeff4 = 0;

	   	 GCB_CoeffPtr[3][1].Coeff5 = 0;

	   	 GCB_CoeffPtr[3][1].Coeff6 = 0;

	   	 GCB_CoeffPtr[3][1].Coeff7 = 0;

	   	GCB_CoeffPtr[3][2].Coeff0 = 0x00000000;

	   		   	 GCB_CoeffPtr[3][2].Coeff1 = 0x00000ff4;

	   		   	 GCB_CoeffPtr[3][2].Coeff2 = 0x00000feb;

	   		   	 GCB_CoeffPtr[3][2].Coeff3 = 0x00000ffe;

	   		   	 GCB_CoeffPtr[3][2].Coeff4 = 0;

	   		   	 GCB_CoeffPtr[3][2].Coeff5 = 0;

	   		   	 GCB_CoeffPtr[3][2].Coeff6 = 0;

	   		   	 GCB_CoeffPtr[3][2].Coeff7 = 0;

	   		   	 GCB_CoeffPtr[3][3].Coeff0 = 0x00000000;

	   		   	 GCB_CoeffPtr[3][3].Coeff1 = 0x00000010;

	   		   	 GCB_CoeffPtr[3][3].Coeff2 = 0x0000000f;

	   		   	 GCB_CoeffPtr[3][3].Coeff3 = 0x00000016;

	   		   	 GCB_CoeffPtr[3][3].Coeff4 = 0;

	   		   	 GCB_CoeffPtr[3][3].Coeff5 = 0;

	   		   	 GCB_CoeffPtr[3][3].Coeff6 = 0;

	   		   	 GCB_CoeffPtr[3][3].Coeff7 = 0;

	   	 TSCB_CoeffPtr[0][0].Coeff0 = 0x00000080;

	   	 TSCB_CoeffPtr[0][0].Coeff1 = 0x00000038;

	   	 TSCB_CoeffPtr[0][0].Coeff2 = 0x0000002c;

	   	 TSCB_CoeffPtr[0][0].Coeff3 = 0x00000051;

	   	 TSCB_CoeffPtr[0][0].Coeff4 = 0x00000035;

	   	 TSCB_CoeffPtr[0][0].Coeff5 = 0x00000006;

	   	 TSCB_CoeffPtr[0][0].Coeff6 = 0x00000023;

	   	 TSCB_CoeffPtr[0][0].Coeff7 = 0x00000035;

	   	 TSCB_CoeffPtr[0][1].Coeff0 = 0x0000000f;

	   	 TSCB_CoeffPtr[0][1].Coeff1 = 0x0000000b;

	   	 TSCB_CoeffPtr[0][1].Coeff2 = 0x0000000c;

	   	 TSCB_CoeffPtr[0][1].Coeff3 = 0x0000001f;

	   	 TSCB_CoeffPtr[0][1].Coeff4 = 0x0000001a;

	   	 TSCB_CoeffPtr[0][1].Coeff5 = 0x0000001e;

	   	 TSCB_CoeffPtr[0][1].Coeff6 = 0x00000004;

	   	 TSCB_CoeffPtr[0][1].Coeff7 = 0x0000000e;

	   	TSCB_CoeffPtr[0][2].Coeff0 = 0x0000004d;

	   		   	 TSCB_CoeffPtr[0][2].Coeff1 = 0x00000010;

	   		   	 TSCB_CoeffPtr[0][2].Coeff2 = 0x00000029;

	   		   	 TSCB_CoeffPtr[0][2].Coeff3 = 0x00000054;

	   		   	 TSCB_CoeffPtr[0][2].Coeff4 = 0x00000069;

	   		   	 TSCB_CoeffPtr[0][2].Coeff5 = 0x00000060;

	   		   	 TSCB_CoeffPtr[0][2].Coeff6 = 0x00000004;

	   		   	 TSCB_CoeffPtr[0][2].Coeff7 = 0x0000003a;

	   		   	 TSCB_CoeffPtr[0][3].Coeff0 = 0x00000032;

	   		   	 TSCB_CoeffPtr[0][3].Coeff1 = 0x00000026;

	   		   	 TSCB_CoeffPtr[0][3].Coeff2 = 0x00000038;

	   		   	 TSCB_CoeffPtr[0][3].Coeff3 = 0x00000003;

	   		   	 TSCB_CoeffPtr[0][3].Coeff4 = 0x00000077;

	   		   	 TSCB_CoeffPtr[0][3].Coeff5 = 0x00000038;

	   		   	 TSCB_CoeffPtr[0][3].Coeff6 = 0x0000004e;

	   		   	 TSCB_CoeffPtr[0][3].Coeff7 = 0x00000080;

	   	 TSCB_CoeffPtr[1][0].Coeff0 = 0x00000004;

	   	 TSCB_CoeffPtr[1][0].Coeff1 = 0x0000001e;

	   	 TSCB_CoeffPtr[1][0].Coeff2 = 0x00000011;

	   	 TSCB_CoeffPtr[1][0].Coeff3 = 0x0000000f;

	   	 TSCB_CoeffPtr[1][0].Coeff4 = 0x0000000c;

	   	 TSCB_CoeffPtr[1][0].Coeff5 = 0x0000000f;

	   	 TSCB_CoeffPtr[1][0].Coeff6 = 0x0000001b;

	   	 TSCB_CoeffPtr[1][0].Coeff7 = 0x0000001f;

	   	 TSCB_CoeffPtr[1][1].Coeff0 = 0x00000008;

	   	 TSCB_CoeffPtr[1][1].Coeff1 = 0x0000001c;

	   	 TSCB_CoeffPtr[1][1].Coeff2 = 0x00000019;

	   	 TSCB_CoeffPtr[1][1].Coeff3 = 0x00000005;

	   	 TSCB_CoeffPtr[1][1].Coeff4 = 0x00000018;

	   	 TSCB_CoeffPtr[1][1].Coeff5 = 0x00000009;

	   	 TSCB_CoeffPtr[1][1].Coeff6 = 0x00000006;

	   	 TSCB_CoeffPtr[1][1].Coeff7 = 0x0000001f;
	   	TSCB_CoeffPtr[1][2].Coeff0 = 0x00000050;

	   		   	 TSCB_CoeffPtr[1][2].Coeff1 = 0x0000002c;

	   		   	 TSCB_CoeffPtr[1][2].Coeff2 = 0x0000002f;

	   		   	 TSCB_CoeffPtr[1][2].Coeff3 = 0x0000003d;

	   		   	 TSCB_CoeffPtr[1][2].Coeff4 = 0x00000057;

	   		   	 TSCB_CoeffPtr[1][2].Coeff5 = 0x00000027;

	   		   	 TSCB_CoeffPtr[1][2].Coeff6 = 0x00000006;

	   		   	 TSCB_CoeffPtr[1][2].Coeff7 = 0x0000004d;

	   		   	 TSCB_CoeffPtr[1][3].Coeff0 = 0x00000006;

	   		   	 TSCB_CoeffPtr[1][3].Coeff1 = 0x00000004;

	   		   	 TSCB_CoeffPtr[1][3].Coeff2 = 0x00000030;

	   		   	 TSCB_CoeffPtr[1][3].Coeff3 = 0x0000002e;

	   		   	 TSCB_CoeffPtr[1][3].Coeff4 = 0x00000020;

	   		   	 TSCB_CoeffPtr[1][3].Coeff5 = 0x00000045;

	   		   	 TSCB_CoeffPtr[1][3].Coeff6 = 0x00000036;

	   		   	 TSCB_CoeffPtr[1][3].Coeff7 = 0x00000036;

	   	 TSCB_CoeffPtr[2][0].Coeff0 = 0x00000023;

	   	 TSCB_CoeffPtr[2][0].Coeff1 = 0x00000004;

	   	 TSCB_CoeffPtr[2][0].Coeff2 = 0x00000014;

	   	 TSCB_CoeffPtr[2][0].Coeff3 = 0x00000034;

	   	 TSCB_CoeffPtr[2][0].Coeff4 = 0x00000058;

	   	 TSCB_CoeffPtr[2][0].Coeff5 = 0x00000052;

	   	 TSCB_CoeffPtr[2][0].Coeff6 = 0x00000018;

	   	 TSCB_CoeffPtr[2][0].Coeff7 = 0x00000042;

	   	 TSCB_CoeffPtr[2][1].Coeff0 = 0x00000032;

	   	 TSCB_CoeffPtr[2][1].Coeff1 = 0x00000038;

	   	 TSCB_CoeffPtr[2][1].Coeff2 = 0x0000001d;

	   	 TSCB_CoeffPtr[2][1].Coeff3 = 0x00000029;

	   	 TSCB_CoeffPtr[2][1].Coeff4 = 0x00000006;

	   	 TSCB_CoeffPtr[2][1].Coeff5 = 0x00000035;

	   	 TSCB_CoeffPtr[2][1].Coeff6 = 0x00000016;

	   	 TSCB_CoeffPtr[2][1].Coeff7 = 0x00000018;
	   	TSCB_CoeffPtr[2][2].Coeff0 = 0x00000028;

	   		   	 TSCB_CoeffPtr[2][2].Coeff1 = 0x00000030;

	   		   	 TSCB_CoeffPtr[2][2].Coeff2 = 0x00000003;

	   		   	 TSCB_CoeffPtr[2][2].Coeff3 = 0x0000002e;

	   		   	 TSCB_CoeffPtr[2][2].Coeff4 = 0x0000001b;

	   		   	 TSCB_CoeffPtr[2][2].Coeff5 = 0x00000037;

	   		   	 TSCB_CoeffPtr[2][2].Coeff6 = 0x0000000c;

	   		   	 TSCB_CoeffPtr[2][2].Coeff7 = 0x00000021;

	   		   	 TSCB_CoeffPtr[2][3].Coeff0 = 0x000001ff;

	   		   	 TSCB_CoeffPtr[2][3].Coeff1 = 0x000001ff;

	   		   	 TSCB_CoeffPtr[2][3].Coeff2 = 0x000001ff;

	   		   	 TSCB_CoeffPtr[2][3].Coeff3 = 0x000001ff;

	   		   	 TSCB_CoeffPtr[2][3].Coeff4 = 0x000001ff;

	   		   	 TSCB_CoeffPtr[2][3].Coeff5 = 0x00000003;

	   		   	 TSCB_CoeffPtr[2][3].Coeff6 = 0x000001ff;

	   		   	 TSCB_CoeffPtr[2][3].Coeff7 = 0x000001ff;

	   	 TSCB_CoeffPtr[3][0].Coeff0 = 0x000001ff;

	   	 TSCB_CoeffPtr[3][0].Coeff1 = 0x000001ff;

	   	 TSCB_CoeffPtr[3][0].Coeff2 = 0x000001ff;

	   	 TSCB_CoeffPtr[3][0].Coeff3 = 0x000001ff;

	   	 TSCB_CoeffPtr[3][0].Coeff4 = 0x000001ff;

	   	 TSCB_CoeffPtr[3][0].Coeff5 = 0x000001ff;

	   	 TSCB_CoeffPtr[3][0].Coeff6 = 0x000001ff;

	   	 TSCB_CoeffPtr[3][0].Coeff7 = 0x00000003;

	   	 TSCB_CoeffPtr[3][1].Coeff0 = 0x000001ff;

	   	 TSCB_CoeffPtr[3][1].Coeff1 = 0x000001ff;

	   	 TSCB_CoeffPtr[3][1].Coeff2 = 0x000001ff;

	   	 TSCB_CoeffPtr[3][1].Coeff3 = 0x000001ff;

	   	 TSCB_CoeffPtr[3][1].Coeff4 = 0x00000003;

	   	 TSCB_CoeffPtr[3][1].Coeff5 = 0x000001ff;

	   	 TSCB_CoeffPtr[3][1].Coeff6 = 0x000001ff;

	   	 TSCB_CoeffPtr[3][1].Coeff7 = 0x000001ff;
	   	TSCB_CoeffPtr[3][2].Coeff0 = 0x00000003;

	   		   	 TSCB_CoeffPtr[3][2].Coeff1 = 0x000001ff;

	   		   	 TSCB_CoeffPtr[3][2].Coeff2 = 0x000001ff;

	   		   	 TSCB_CoeffPtr[3][2].Coeff3 = 0x000001ff;

	   		   	 TSCB_CoeffPtr[3][2].Coeff4 = 0x000001ff;

	   		   	 TSCB_CoeffPtr[3][2].Coeff5 = 0x000001ff;

	   		   	 TSCB_CoeffPtr[3][2].Coeff6 = 0x000001ff;

	   		   	 TSCB_CoeffPtr[3][2].Coeff7 = 0x000001ff;

	   		   	 TSCB_CoeffPtr[3][3].Coeff0 = 0x000001ff;

	   		   	 TSCB_CoeffPtr[3][3].Coeff1 = 0x000001ff;

	   		   	 TSCB_CoeffPtr[3][3].Coeff2 = 0x000001ff;

	   		   	 TSCB_CoeffPtr[3][3].Coeff3 = 0x00000003;

	   		   	 TSCB_CoeffPtr[3][3].Coeff4 = 0x000001ff;

	   		   	 TSCB_CoeffPtr[3][3].Coeff5 = 0x00000003;

	   		   	 TSCB_CoeffPtr[3][3].Coeff6 = 0x000001ff;

	   		   	 TSCB_CoeffPtr[3][3].Coeff7 = 0x000001ff;

	   		  usleep(1);
	   		  	 for(u32 i=0;i<4;i++){
	   		  		for(u32 j=0;j<4;j=j+1){
	   		  			status = XRFdc_SetCalCoefficients(&InstancePtr_RFSOC,i,j,XRFDC_CAL_BLOCK_GCB, &GCB_CoeffPtr[i][j]);
	   		  			status = XRFdc_SetCalCoefficients(&InstancePtr_RFSOC,i,j,XRFDC_CAL_BLOCK_TSCB, &TSCB_CoeffPtr[i][j]);
	   		  		//	status = XRFdc_SetCalCoefficients(&InstancePtr_RFSOC,i,j,XRFDC_CAL_BLOCK_OCB2, &OB2_CoeffPtr[i][j]);
	   		 }
	   	 }

#endif



	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* libmetal 鏃ュ織杈撳嚭鍥炶皟銆�
*
* RFdc MTS 鐩稿叧椹卞姩浠ｇ爜浼氶�氳繃 metal_log() 杈撳嚭璋冭瘯淇℃伅銆�
* 杩欓噷鎶� libmetal 鐨勬棩蹇楅噸瀹氬悜鍒� UART锛屽苟鎶婂崟鐙殑 '\n' 杞崲鎴� '\r\n'锛�
* 浣夸覆鍙ｇ粓绔樉绀烘洿瑙勬暣銆�
*
******************************************************************************/
void my_metal_default_log_handler(enum metal_log_level level, const char *format, ...) {
 	char msg[1024];
 	char msgOut[1048];
 	char *outPtr;
 	int i;

 	va_list args;

 	static const char *level_strs[] = {
 	"metal: emergency: ",
 	"metal: alert:     ",
 	"metal: critical:  ",
 	"metal: error:     ",
 	"metal: warning:   ",
 	"metal: notice:    ",
 	"metal: info:      ",
 	"metal: debug:     ",
 	};

 	va_start(args, format);
 	vsnprintf(msg, sizeof(msg), format, args);
 	va_end(args);

 	//replace single \n with \n\r
 	outPtr = msgOut;
 	for(i=0; i<1024; i++) {
 		// if /n/r or /r/n combo
 		if ((msg[i] == '\r' && msg[i+1] == '\n') || (msg[i] == '\n' && msg[i+1] == '\r')) {
 			*outPtr++ = msg[i++];
 		} else if(msg[i] == '\n') {
 			//if first char in string is \n, then remove
 			if(i==0) {
 				continue;
 			} else {
 				*outPtr++ = '\r';

 			}
 		}

 		*outPtr++ = msg[i];

 		if(msg[i] == 0) {
 			break;
 		}
 	}

 	//if line doesn't end with \n\r, then add it
 	if( (msg[i-1] != '\n') && (msg[i-1] != '\r') ) {
 		*(outPtr-1) = '\r';
 		*outPtr++ = '\n';
 		*outPtr++ = 0;
 	}

 	if (level <= METAL_LOG_EMERGENCY || level > METAL_LOG_DEBUG)
 		level = METAL_LOG_EMERGENCY;

 	xil_printf("%s%s", level_strs[level], msgOut);



 }

 int metal_log_init(void)
 {
 #ifndef __BAREMETAL__
 	struct metal_device *deviceptr;
 #endif

 	/*
 	 * 鍒濆鍖� libmetal 鏃ュ織绯荤粺銆�
 	 * bare-metal 涓嬩笉娉ㄥ唽棰濆 metal device锛屽彧璁剧疆鏃ュ織鍥炶皟鍜屾棩蹇楃瓑绾с��
 	 */
 	// struct metal_init_params init_param = METAL_INIT_DEFAULTS;
 	struct metal_init_params init_param = {\
 			.log_handler= my_metal_default_log_handler,\
 			.log_level= METAL_LOG_DEBUG,\
 	};

 	if (metal_init(&init_param)) {
 		printf("ERROR: Failed to run metal initialization\n");
 		return XRFDC_FAILURE;
 	}
 	metal_set_log_level(METAL_LOG_DEBUG);


 #ifndef __BAREMETAL__
 	status = XRFdc_RegisterMetal(RFdcInstPtr, RFdcDeviceId, &deviceptr);
 	if (status != XRFDC_SUCCESS) {
 		return XRFDC_FAILURE;
 	}
 #endif
 	return XST_SUCCESS ;
 }
