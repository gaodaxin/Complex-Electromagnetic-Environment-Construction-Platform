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
#include "platform.h"
#include "xil_printf.h"
#include "mio_gpio.h"
#include "xparameters.h"
#include "xstatus.h"
#include "xplatform_info.h"
#include "xrfdc.h"
#include "sleep.h"
#include "hmc_7044.h"
#include "helloworld.h"


#include "emif.h"




XRFdc_Cal_Freeze_Settings CalFreezePtr;


#define  SPI_SclkSdioId  XPAR_XGPIOPS_0_DEVICE_ID
#define  SPI_CsId        XPAR_XGPIOPS_0_DEVICE_ID












#define  DAC_RF_OPEN_ADDR		(0x04)
#define  DAC_RF_EN_ADDR			(0x05)
#define  DAC_DATA_SEL_ADDR		(0x09)




#define  AUTO_CAL_MODE			(0)
#define  CAL_MODE1				(1)
#define  CAL_MODE2				(2)


#define  RF_ADC					(0)
#define  RF_DAC					(1)




#define  NyquistZone_ODD		(1)
#define  NyquistZone_Even		(2)



#ifdef __BAREMETAL__
#define RFDC_DEVICE_ID 	XPAR_XRFDC_0_DEVICE_ID
#define I2CBUS	1
#define XRFDC_BASE_ADDR		XPAR_XRFDC_0_BASEADDR
#define RFDC_DEV_NAME    XPAR_XRFDC_0_DEV_NAME
#else
#define RFDC_DEVICE_ID 	0
#define I2CBUS	12
#endif

/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions *******************/

/************************** Function Prototypes ****************************/


/************************** Variable Definitions **************************/

/*
 * The following are declared globally so they are zeroed and can be
 * easily accessible from a debugger.
 */


#define    ADC_CHANN_NUM        8
#define    MAX_TILE_NUM			4
#define    MAX_BLOCK_NUM		2


XRFdc_Cal_Freeze_Settings   XRFdc_Cal_Freeze_Inst[MAX_TILE_NUM][MAX_BLOCK_NUM]={0};

XRFdc_Mixer_Settings ADC_Mixer_Settings={0};
XRFdc_Mixer_Settings DAC_Mixer_Settings={0};

u32  IMR_Mode  =0;
u32  DataPath_Mode  =0;



XRFdc_QMC_Settings QMCSettingsPtr={0};




XRFdc_DSA_Settings XRFdc_DSA_Struc={0};






//static XRFdc_Calibration_Coefficients OCB1_Calibration_Coefficients={0xffb8ffb3,0xffcfffbd,0x0016fffa,0x0012fffd,0x0000000,0x0000000,0x0000000,0x0000000};
//static XRFdc_Calibration_Coefficients OCB2_Calibration_Coefficients={0x002c0029,0x0046001b,0x001e0020,0x001c003b,0x0000000,0x0000000,0x0000000,0x0000000};
//static XRFdc_Calibration_Coefficients GCB_Calibration_Coefficients={0x0ffa0ffd,0x0fde0fb4,0x0fbe0fb7,0x0ff70ffb,0x0000000,0x0000000,0x0000000,0x0000000};
//static XRFdc_Calibration_Coefficients TSCB_Calibration_Coefficients={0x000c001f,0x0042002d,0x0037004b,0x0004004b,0x000d0045,0x0036005d,0x004c007a,0x0029005d};
//XRFdc_Calibration_Coefficients *CoeffPtr=&Calibration_Coefficients;

static XRFdc_Calibration_Coefficients OCB1_Calibration_Coefficients[ADC_CHANN_NUM] = {{0xffe7ffc3,0xffdeffb4,0x00190016,0x000a0004,0x00000000,0x00000000,0x00000000,0x00000000},
																					  {0xffd6ffc1,0xffcaffba,0x0007ffff,0x0002ffff,0x00000000,0x00000000,0x00000000,0x00000000},
																					  {0xffcfffd6,0xfff5ffd6,0x00120019,0x00180015,0x00000000,0x00000000,0x00000000,0x00000000},
																					  {0xffb8ffb3,0xffcfffbd,0x0016fffa,0x0012fffd,0x00000000,0x00000000,0x00000000,0x00000000},
																					  {0xffb8ffb3,0xffcfffbd,0x0016fffa,0x0012fffd,0x00000000,0x00000000,0x00000000,0x00000000},
																					  {0xffb8ffb3,0xffcfffbd,0x0016fffa,0x0012fffd,0x00000000,0x00000000,0x00000000,0x00000000},
																					  {0x00370019,0x0032003a,0x003a0034,0x003d0039,0x00000000,0x00000000,0x00000000,0x00000000},
																					  {0xffd5ffd3,0x005d0055,0x0020005c,0xffd7fff7,0x00000000,0x00000000,0x00000000,0x00000000}
																					  };
static XRFdc_Calibration_Coefficients OCB2_Calibration_Coefficients[ADC_CHANN_NUM] = {{0x0004fffb,0x0011ffff,0xfff6000e,0xffff0022,0x00000000,0x00000000,0x00000000,0x00000000},
																					  {0xff85ff76,0xff84ff94,0xff65ff87,0xff6bff94,0x00000000,0x00000000,0x00000000,0x00000000},
																					  {0x002d0032,0x00300028,0x002c003b,0x001e0039,0x00000000,0x00000000,0x00000000,0x00000000},
																					  {0x002c0029,0x0046001b,0x001e0020,0x001c003b,0x00000000,0x00000000,0x00000000,0x00000000},
																					  {0x002c0029,0x0046001b,0x001e0020,0x001c003b,0x00000000,0x00000000,0x00000000,0x00000000},
																					  {0x002c0029,0x0046001b,0x001e0020,0x001c003b,0x00000000,0x00000000,0x00000000,0x00000000},
																					  {0x00260001,0x00200fee,0x0fe9000d,0x0010002b,0x00000000,0x00000000,0x00000000,0x00000000},
																					  {0x00070000,0x0ffd0001,0x000a0017,0x000a000b,0x00000000,0x00000000,0x00000000,0x00000000}
																					  };
static XRFdc_Calibration_Coefficients GCB_Calibration_Coefficients [ADC_CHANN_NUM] = {{0x00190000,0x0ff90fd7,0x00050fbf,0x0ffa0fed,0x00000000,0x00000000,0x00000000,0x00000000},
																					  {0x0fca0000,0x0feb0fe6,0x0fdf0fec,0x0fc90fe2,0x00000000,0x00000000,0x00000000,0x00000000},
																					  {0x0fe00000,0x00120024,0x00170012,0x0fff0ffe,0x00000000,0x00000000,0x00000000,0x00000000},
																					  {0x0ffa0ffd,0x0fde0fb4,0x0fbe0fb7,0x0ff70ffb,0x00000000,0x00000000,0x00000000,0x00000000},
																					  {0x0ffa0ffd,0x0fde0fb4,0x0fbe0fb7,0x0ff70ffb,0x00000000,0x00000000,0x00000000,0x00000000},
																					  {0x0ffa0ffd,0x0fde0fb4,0x0fbe0fb7,0x0ff70ffb,0x00000000,0x00000000,0x00000000,0x00000000},
																					  {0x001c0007,0x00110036,0x00130034,0x00040036,0x001d001a,0x0010003f,0x0021002f,0x00140042},
																					  {0x000b0015,0x001b0010,0x00240005,0x0037002d,0x0028001a,0x001b0028,0x00370015,0x003e0026}
																					  };
static XRFdc_Calibration_Coefficients TSCB_Calibration_Coefficients[ADC_CHANN_NUM] = {{0x003a0008,0x0045000a,0x00310003,0x0039000e,0x00250014,0x00470020,0x00430009,0x00510010},
																					  {0x0048004f,0x002a003b,0x00650026,0x000e005a,0x00360057,0x00300037,0x0055002a,0x00040069},
																					  {0x00320041,0x0004003d,0x001f0009,0x003c0028,0x00390037,0x00060055,0x0056001a,0x00320030},
																					  {0x000c001f,0x0042002d,0x0037004b,0x0004004b,0x000d0045,0x0036005d,0x004c007a,0x0029005d},
																					  {0x000c001f,0x0042002d,0x0037004b,0x0004004b,0x000d0045,0x0036005d,0x004c007a,0x0029005d},
																					  {0x000c001f,0x0042002d,0x0037004b,0x0004004b,0x000d0045,0x0036005d,0x004c007a,0x0029005d},
																					  {0x80040000,0x00000000,0x00025c50,0x00000000,0x0002b4b8,0x00000000,0x00000001,0x00000000},
																					  {0x80040000,0x00000000,0x00029c08,0x00000000,0x00040000,0x00000000,0xffffffff,0x00000000}
																					};
//static XRFdc_Calibration_Coefficients OCB1_Calibration_Coefficients[ADC_CHANN_NUM] = {{0xffe7ffc3,0xffdeffb4,0x00190016,0x000a0004,0x00000000,0x00000000,0x00000000,0x00000000},
//																					  {0xffd2ffd1,0xffbfffba,0x001a000a,0x0009002f,0x0000000,0x0000000,0x0000000,0x0000000},
//																					  {0xffe7ffc3,0xffdeffb4,0x00190016,0x000a0004,0x0000000,0x0000000,0x0000000,0x0000000},
//																					  {0xffd6ffc1,0xffcaffba,0x0007ffff,0x0002ffff,0x0000000,0x0000000,0x0000000,0x0000000},
//																					  {0xffcfffd6,0xfff5ffd6,0x00120019,0x00180015,0x0000000,0x0000000,0x0000000,0x0000000},
//																					  {0xffb8ffb3,0xffcfffbd,0x0016fffa,0x0012fffd,0x0000000,0x0000000,0x0000000,0x0000000},
//																					  {0xffb8ffb3,0xffcfffbd,0x0016fffa,0x0012fffd,0x0000000,0x0000000,0x0000000,0x0000000},
//																					  {0xffb8ffb3,0xffcfffbd,0x0016fffa,0x0012fffd,0x0000000,0x0000000,0x0000000,0x0000000}
//																					  };
//static XRFdc_Calibration_Coefficients OCB2_Calibration_Coefficients[ADC_CHANN_NUM] = //{{0x00320018,0x0021001e,0x002a003c,0x00150037,0x0000000,0x0000000,0x0000000,0x0000000},
//																					 {{0x0004fffb,0x0011ffff,0xfff6000e,0xffff0022,0x00000000,0x00000000,0x00000000,0x00000000},
//																					  {0xffd5ffd3,0x005d0055,0x0020005c,0xffd7fff7,0x0000000,0x0000000,0x0000000,0x0000000},
//																					  {0x0004fffb,0x0011ffff,0xfff6000e,0xffff0022,0x0000000,0x0000000,0x0000000,0x0000000},
//																					  {0xff85ff76,0xff84ff94,0xff65ff87,0xff6bff94,0x0000000,0x0000000,0x0000000,0x0000000},
//																					  {0x002d0032,0x00300028,0x002c003b,0x001e0039,0x0000000,0x0000000,0x0000000,0x0000000},
//																					  {0x002c0029,0x0046001b,0x001e0020,0x001c003b,0x0000000,0x0000000,0x0000000,0x0000000},
//																					  {0x002c0029,0x0046001b,0x001e0020,0x001c003b,0x0000000,0x0000000,0x0000000,0x0000000},
//																					  {0x002c0029,0x0046001b,0x001e0020,0x001c003b,0x0000000,0x0000000,0x0000000,0x0000000}
//																					  };
//static XRFdc_Calibration_Coefficients GCB_Calibration_Coefficients [ADC_CHANN_NUM] = {{0x00190000, 0x0ff90fd7,0x00050fbf,0x0ffa0fed,0x00000000,0x00000000,0x00000000,0x00000000},
//																					  {0x00070000,0x0ffd0001,0x000a0017,0x000a000b,0x0000000,0x0000000,0x0000000,0x0000000},
//																					  {0x00190000,0x0ff90fd7,0x00050fbf,0x0ffa0fed,0x0000000,0x0000000,0x0000000,0x0000000},
//																					  {0x0fca0000,0x0feb0fe6,0x0fdf0fec,0x0fc90fe2,0x0000000,0x0000000,0x0000000,0x0000000},
//																					  {0x0fe00000,0x00120024,0x00170012,0x0fff0ffe,0x0000000,0x0000000,0x0000000,0x0000000},
//																					  {0x0ffa0ffd,0x0fde0fb4,0x0fbe0fb7,0x0ff70ffb,0x0000000,0x0000000,0x0000000,0x0000000},
//																					  {0x0ffa0ffd,0x0fde0fb4,0x0fbe0fb7,0x0ff70ffb,0x0000000,0x0000000,0x0000000,0x0000000},
//																					  {0x0ffa0ffd,0x0fde0fb4,0x0fbe0fb7,0x0ff70ffb,0x0000000,0x0000000,0x0000000,0x0000000}
//																					  };
//static XRFdc_Calibration_Coefficients TSCB_Calibration_Coefficients[ADC_CHANN_NUM] = {{0x003a0008,0x0045000a,0x00310003,0x0039000e,0x00250014,0x00470020,0x00430009,0x00510010},
//																					  {0x000b0015,0x001b0010,0x00240005,0x0037002d,0x0028001a,0x001b0028,0x00370015,0x003e0026},
//																					  {0x003a0008,0x0045000a,0x00310003,0x0039000e,0x00250014,0x00470020,0x00430009,0x00510010},
//																					  {0x0048004f,0x002a003b,0x00650026,0x000e005a,0x00360057,0x00300037,0x0055002a,0x00040069},
//																					  {0x00320041,0x0004003d,0x001f0009,0x003c0028,0x00390037,0x00060055,0x0056001a,0x00320030},
//																					  {0x000c001f,0x0042002d,0x0037004b,0x0004004b,0x000d0045,0x0036005d,0x004c007a,0x0029005d},
//																					  {0x000c001f,0x0042002d,0x0037004b,0x0004004b,0x000d0045,0x0036005d,0x004c007a,0x0029005d},
//																					  {0x000c001f,0x0042002d,0x0037004b,0x0004004b,0x000d0045,0x0036005d,0x004c007a,0x0029005d}
//																					};



/***************** Macros (Inline Functions) Definitions ********************/
#ifdef __BAREMETAL__
#define printf xil_printf
#endif
/************************** Function Prototypes *****************************/

static int RFSoc_Sys_Config(u16 SysMonDeviceId,u16  CAL_Mode_Sel);
static int CompareFabricRate(u32 SetFabricRate, u32 GetFabricRate);

/************************** Variable Definitions ****************************/

static XRFdc RFdcInst;      /* RFdc driver instance */
struct metal_device *deviceptr = NULL;

u32     bus_test_Data =0x00;










void  MTS_SYNC_Function(u16 RFdcDeviceId )
{

	int status_adc;
	int status_dac=0;
	int status=0;
	int i;
	u32  factor=0;
	u32 SysRefEnable = 0;
	XRFdc_Mixer_Settings Mixer_Settings; // declare mixer settings struct
	u32 Type = XRFDC_ADC_TILE;
	u32 Tile_Id ;
	u32 Block_Id ;
	XRFdc_Config *ConfigPtr;
	XRFdc *RFdcInstPtr = &RFdcInst;


	struct metal_init_params init_param = METAL_INIT_DEFAULTS;

	if (metal_init(&init_param)) {
		printf("ERROR: Failed to run metal initialization\n");
		return XRFDC_FAILURE;
	}
	metal_set_log_level(METAL_LOG_DEBUG);
    ConfigPtr = XRFdc_LookupConfig(RFdcDeviceId);
    if (ConfigPtr == NULL) {
		return XRFDC_FAILURE;
	}

	status = XRFdc_RegisterMetal(RFdcInstPtr, RFdcDeviceId, &deviceptr);
	if (status != XRFDC_SUCCESS) {
		return XRFDC_FAILURE;
	}


    status = XRFdc_CfgInitialize(RFdcInstPtr, ConfigPtr);
    if (status != XRFDC_SUCCESS) {
        printf("RFdc Init Failure\n\r");
    }

    printf("=== RFdc Initialized - Running Multi-tile Sync ===\r\t\n");
	XRFdc_MultiConverter_Sync_Config Adc_Sync_Config; // Declare DAC MTS Settings struct
	status=XRFdc_MultiConverter_Init(&Adc_Sync_Config, 0, 0, XRFDC_TILE_ID1); // InitializeADC MTS Settings REF sysref TILE2

	Adc_Sync_Config.Tiles = 0x0f; // Tiles to sync bit-mask: DAC tiles 0 to 3
	Adc_Sync_Config.Target_Latency = -1; // Target latency = 280 (measuredvalue) + 16 (margin), set to -1 if unknown
	Adc_Sync_Config.Target_Latency = 112; // Target latency = 280 (measuredvalue) + 16 (margin), set to -1 if unknown
	status_adc=XRFdc_MultiConverter_Sync(RFdcInstPtr,XRFDC_ADC_TILE,&Adc_Sync_Config); // Run Multi-Tile-Sync for the DAC Group
	if((status_adc!=XRFDC_MTS_OK)) //{Execute error code;} and Report detailson synchronization (optional)
	for(i=0; i<4; i++) {
	  if((1<<i)&Adc_Sync_Config.Tiles) {
	XRFdc_GetInterpolationFactor(RFdcInstPtr, i, 0, &factor);
	xil_printf(" ADC %d: Latency(T1) =%3d, Adjusted Delay(T%d) =%3d\n", i, Adc_Sync_Config.Latency[i], factor,
	Adc_Sync_Config.Offset[i]);
	}
	}

	XRFdc_MultiConverter_Sync_Config Dac_Sync_Config;
	status=XRFdc_MultiConverter_Init(&Dac_Sync_Config, 0, 0, XRFDC_TILE_ID2);//  InitializeDAC MTS Settings REF sysref TILE1
	Dac_Sync_Config.Tiles = 0x0F;
	Dac_Sync_Config.Target_Latency=272;
//	Dac_Sync_Config.Target_Latency=-1;

	status_dac=XRFdc_MultiConverter_Sync(RFdcInstPtr,XRFDC_DAC_TILE,&Dac_Sync_Config);
	if((status_dac!=XRFDC_MTS_OK)) //{Execute error code;} and Report detailson synchronization (optional)
	for(i=0; i<4; i++) {
	  if((1<<i)&Dac_Sync_Config.Tiles) {
	XRFdc_GetInterpolationFactor(RFdcInstPtr, i, 0, &factor);
	xil_printf(" DAC%d: Latency(T1) =%3d, Adjusted Delay(T%d) =%3d\n", i, Dac_Sync_Config.Latency[i], factor,
	Dac_Sync_Config.Offset[i]);
	}
	}








	//disable sysref capture
	SysRefEnable = 0;
//	DevFpgaWrite(0x02,0x00);
	status_adc|=XRFdc_MTS_Sysref_Config(RFdcInstPtr, &Dac_Sync_Config,&Adc_Sync_Config, SysRefEnable);
	//2. Arm the mixer settings, NCO phase reset, QMC, and/or coarse delay.
	Mixer_Settings.EventSource = XRFDC_EVNT_SRC_SYSREF;
	Mixer_Settings.Freq = 0.0;
	Mixer_Settings.PhaseOffset = 0;
	Mixer_Settings.CoarseMixFreq = XRFDC_COARSE_MIX_OFF;
	Mixer_Settings.MixerMode = XRFDC_MIXER_MODE_R2R;
	Mixer_Settings.FineMixerScale = XRFDC_MIXER_SCALE_AUTO;
	Mixer_Settings.MixerType = XRFDC_MIXER_TYPE_FINE;
	// it is assumed other Mixer settings have been previously assigned in
	for (Tile_Id = 0; Tile_Id < MAX_TILE_NUM; Tile_Id++ ) {
	for (Block_Id = 0; Block_Id < MAX_BLOCK_NUM; Block_Id++ ) {
	status=XRFdc_SetMixerSettings(RFdcInstPtr, Type, Tile_Id, Block_Id,
	&Mixer_Settings);
	status_adc |= XRFdc_ResetNCOPhase(RFdcInstPtr,Type,
	Tile_Id,Block_Id);
	//note that Coarse mixer and QMC could also be part of this for loop.
	XRFdc_UpdateEvent(RFdcInstPtr, XRFDC_ADC_TILE, Tile_Id, Block_Id, XRFDC_EVENT_MIXER);
	}
	}

	if (status_adc!=XST_SUCCESS)
	{
		xil_printf("NCO Seteing Failed!!!\n\n\n");
	}

	//enable  sysref capture
	SysRefEnable = 1;
	DevFpgaWrite(0x02,0x01);
	status_adc|=XRFdc_MTS_Sysref_Config(RFdcInstPtr, &Dac_Sync_Config,&Adc_Sync_Config, SysRefEnable);

	usleep(1000*1000);/////wait 1 second
	//enable  sysref capture
	SysRefEnable = 0;
	DevFpgaWrite(0x02,0x00);
	status_adc|=XRFdc_MTS_Sysref_Config(RFdcInstPtr, &Dac_Sync_Config,&Adc_Sync_Config, SysRefEnable);
}

#ifdef __BAREMETAL__
metal_phys_addr_t metal_phys = XRFDC_BASE_ADDR;
static struct metal_device CustomDev = {
	/* RFdc device */
	.name = RFDC_DEV_NAME,
	.bus = NULL,
	.num_regions = 1,
	.regions = {
		{
			.virt = (void *)XRFDC_BASE_ADDR,
			.physmap = &metal_phys,
			.size = 0x40000,
			.page_shift = (unsigned)(-1),
			.page_mask = (unsigned)(-1),
			.mem_flags = 0x0,
			.ops = {NULL},
		}
	},
	.node = {NULL},
	.irq_num = 0,
	.irq_info = NULL,
};
#endif
/****************************************************************************/
/**
*
* Main function that invokes the polled example in this file.
*
* @param	None.
*
* @return
*		- XRFDC_SUCCESS if the example has completed successfully.
*		- XRFDC_FAILURE if the example has failed.
*
* @note		None.
*
*****************************************************************************/


/****************************************************************************/
/**
*
* This function runs a test on the RFSoC data converter device using the
* driver APIs.
* This function does the following tasks:
*	- Initialize the RFdc device driver instance
*	- Set the fabric width.
*	- Get the fabric width
*	- Compare Set and Get fabric widths.
*   --CAL_Mode_Sel			0------   锟斤拷锟皆讹拷校准
*   						1-------- 锟斤拷锟斤拷锟皆讹拷校准
*
* @param	RFdcDeviceId is the XPAR_<XRFDC_instance>_DEVICE_ID value
*		from xparameters.h.
*
* @return
*		- XRFDC_SUCCESS if the example has completed successfully.
*		- XRFDC_FAILURE if the example has failed.
*
* @note   	None
*
****************************************************************************/
int RFSoc_Sys_Config(u16 RFdcDeviceId, u16  CAL_Mode_Sel)
{

	int Status;
	u16 Tile;
	u16 Block;
	u8 cal_mode=0;
	XRFdc_Config *ConfigPtr;
	XRFdc *RFdcInstPtr = &RFdcInst;
	u32 ADCSetFabricRate[4];
	u32 DACSetFabricRate[4];
	u32 GetFabricRate;


	struct metal_init_params init_param = METAL_INIT_DEFAULTS;

	if (metal_init(&init_param)) {
		printf("ERROR: Failed to run metal initialization\n");
		return XRFDC_FAILURE;
	}

	/* Initialize the RFdc driver. */
	ConfigPtr = XRFdc_LookupConfig(RFdcDeviceId);
	if (ConfigPtr == NULL) {
		return XRFDC_FAILURE;
	}

	/* Register & MAP RFDC to Libmetal */
#ifdef __BAREMETAL__
	deviceptr = &CustomDev;
#endif

	Status = XRFdc_RegisterMetal(RFdcInstPtr, RFdcDeviceId, &deviceptr);
	if (Status != XRFDC_SUCCESS) {
		return XRFDC_FAILURE;
	}

	/* Initializes the controller */
	Status = XRFdc_CfgInitialize(RFdcInstPtr, ConfigPtr);
	if (Status != XRFDC_SUCCESS) {
		return XRFDC_FAILURE;
	}




	for (int i = 0; i < 4; i++) {
		Status = XRFdc_Reset(RFdcInstPtr, XRFDC_ADC_TILE, i);

	}

	for (int i = 0; i < 4; i++) {
		Status = XRFdc_Reset(RFdcInstPtr, XRFDC_DAC_TILE, i);

	}



	for (int i = 0; i < 4; i++) {
	CalFreezePtr.DisableFreezePin = 0;
	CalFreezePtr.FreezeCalibration = 0;
	printf("tile=%d,block0,SetCalibrationMode,Status=%d\r\n", i,
			Status);
	Status = XRFdc_SetCalFreeze(RFdcInstPtr, i, 0, &CalFreezePtr);
	Status = XRFdc_GetCalFreeze(RFdcInstPtr, i, 0, &CalFreezePtr);
	}

//	DevFpgaWrite(0x0B,0x03);
	// 锟斤拷锟斤拷锟斤拷锟斤拷Tile锟斤拷Block锟斤拷锟斤拷锟斤拷锟斤拷
	for(Tile = 0; Tile < MAX_TILE_NUM; Tile++) {
	    for (Block = 0; Block < MAX_BLOCK_NUM; Block++) {
	        // 锟斤拷锟紻AC锟斤拷锟角凤拷锟斤拷锟斤拷
	        if (XRFdc_IsDACBlockEnabled(RFdcInstPtr, Tile, Block)) {
	            // 锟斤拷取锟斤拷前fabric写锟斤拷锟斤拷效锟斤拷锟斤拷
	            Status = XRFdc_GetFabWrVldWords(RFdcInstPtr, XRFDC_DAC_TILE, Tile, Block, &GetFabricRate);
	            if (Status != XRFDC_SUCCESS) {
	                return XRFDC_FAILURE; // 锟斤拷取失锟杰凤拷锟截达拷锟斤拷
	            }

	            // 锟斤拷锟斤拷fabric锟斤拷锟绞ｏ拷锟斤拷锟桔伙拷取锟斤拷锟斤拷值锟角凤拷锟斤拷锟�0锟斤拷锟斤拷锟矫ｏ拷
	            if(GetFabricRate > 0) {
	                DACSetFabricRate[Block] = GetFabricRate;
	            } else {
	                DACSetFabricRate[Block] = GetFabricRate;
	            }

	            // 锟斤拷锟斤拷fabric写锟斤拷锟斤拷效锟斤拷锟斤拷
	            Status = XRFdc_SetFabWrVldWords(RFdcInstPtr, Tile, Block, DACSetFabricRate[Block]);
	            if (Status != XRFDC_SUCCESS) {
	                return XRFDC_FAILURE; // 锟斤拷锟斤拷失锟杰凤拷锟截达拷锟斤拷
	            }

	            // 锟劫次伙拷取fabric锟斤拷锟斤拷锟斤拷锟斤拷证锟斤拷锟斤拷
	            Status = XRFdc_GetFabWrVldWords(RFdcInstPtr, XRFDC_DAC_TILE, Tile, Block, &GetFabricRate);
	            if (Status != XRFDC_SUCCESS) {
	                return XRFDC_FAILURE; // 锟斤拷取失锟杰凤拷锟截达拷锟斤拷
	            }

	            // 锟饺斤拷锟斤拷锟矫碉拷fabric锟斤拷锟绞猴拷实锟绞伙拷取锟斤拷锟斤拷锟斤拷
	            Status = CompareFabricRate(DACSetFabricRate[Block], GetFabricRate);
	            if (Status != XRFDC_SUCCESS) {
	                return XRFDC_FAILURE; // 锟饺较诧拷匹锟戒返锟截达拷锟斤拷
	            }
	        }

	        // 锟斤拷锟紸DC锟斤拷锟角凤拷锟斤拷锟斤拷
	        if (XRFdc_IsADCBlockEnabled(RFdcInstPtr, Tile, Block)) {
	            // 锟斤拷锟叫Ｗ寄Ｊ窖★拷锟轿�0x01锟斤拷锟皆讹拷校准模式锟斤拷
	            if(CAL_Mode_Sel == 0x01) {
	                // 注锟酵碉拷锟侥达拷锟诫：原锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷校准模式锟斤拷锟截闭猴拷锟斤拷锟斤拷ADC
	                // Status = XRFdc_SetCalibrationMode(RFdcInstPtr, Block, 0, AUTO_CAL_MODE);
	                // Status = XRFdc_GetCalibrationMode(RFdcInstPtr, Block, 0, &cal_mode);
	                // Status = XRFdc_Shutdown(RFdcInstPtr, 0, Block); // 锟截憋拷ADC
	                // Status = XRFdc_StartUp(RFdcInstPtr, 0, Block); // 锟斤拷锟斤拷ADC

	                // 锟截革拷锟斤拷取OCB1锟斤拷偏锟斤拷校准锟斤拷1锟斤拷系锟斤拷锟斤拷锟轿ｏ拷确锟斤拷锟斤拷锟斤拷锟饺讹拷,校准频锟斤拷0.75*Fs+10M=1485M锟斤拷锟斤拷锟斤拷锟轿拷锟阶夹Ｗ硷拷锟斤拷锟斤拷械锟斤拷锟斤拷俅锟斤拷藕锟皆�
	                Status = XRFdc_GetCalCoefficients(RFdcInstPtr, Tile, Block, 0, &OCB1_Calibration_Coefficients[2*Tile+Block]);
	                Status = XRFdc_GetCalCoefficients(RFdcInstPtr, Tile, Block, 0, &OCB1_Calibration_Coefficients[2*Tile+Block]);
	                Status = XRFdc_GetCalCoefficients(RFdcInstPtr, Tile, Block, 0, &OCB1_Calibration_Coefficients[2*Tile+Block]);

	                // 锟截革拷锟斤拷取OCB2锟斤拷偏锟斤拷校准锟斤拷2锟斤拷系锟斤拷锟斤拷锟斤拷
	                Status = XRFdc_GetCalCoefficients(RFdcInstPtr, Tile, Block, 1, &OCB2_Calibration_Coefficients[2*Tile+Block]);
	                Status = XRFdc_GetCalCoefficients(RFdcInstPtr, Tile, Block, 1, &OCB2_Calibration_Coefficients[2*Tile+Block]);
	                Status = XRFdc_GetCalCoefficients(RFdcInstPtr, Tile, Block, 1, &OCB2_Calibration_Coefficients[2*Tile+Block]);

	                // 锟截革拷锟斤拷取GCB锟斤拷锟斤拷锟斤拷校准锟介）系锟斤拷锟斤拷锟斤拷
	                Status = XRFdc_GetCalCoefficients(RFdcInstPtr, Tile, Block, 2, &GCB_Calibration_Coefficients[2*Tile+Block]);
	                Status = XRFdc_GetCalCoefficients(RFdcInstPtr, Tile, Block, 2, &GCB_Calibration_Coefficients[2*Tile+Block]);
	                Status = XRFdc_GetCalCoefficients(RFdcInstPtr, Tile, Block, 2, &GCB_Calibration_Coefficients[2*Tile+Block]);

	                // 锟截革拷锟斤拷取TSCB锟斤拷时锟斤拷偏锟斤拷校准锟介）系锟斤拷锟斤拷锟斤拷
	                Status = XRFdc_GetCalCoefficients(RFdcInstPtr, Tile, Block, 3, &TSCB_Calibration_Coefficients[2*Tile+Block]);
	                Status = XRFdc_GetCalCoefficients(RFdcInstPtr, Tile, Block, 3, &TSCB_Calibration_Coefficients[2*Tile+Block]);
	                Status = XRFdc_GetCalCoefficients(RFdcInstPtr, Tile, Block, 3, &TSCB_Calibration_Coefficients[2*Tile+Block]);
	            }

	            // 注锟酵碉拷锟侥达拷锟诫：锟斤拷锟斤拷校准模式锟斤拷锟斤拷锟斤拷ADC
	            // Status = XRFdc_SetCalibrationMode(RFdcInstPtr, Block, 0, AUTO_CAL_MODE);
	            // Status = XRFdc_GetCalibrationMode(RFdcInstPtr, Block, 0, &cal_mode);
	            // Status = XRFdc_Shutdown(RFdcInstPtr, 0, Block); // 锟截憋拷ADC
	            // Status = XRFdc_StartUp(RFdcInstPtr, 0, Block); // 锟斤拷锟斤拷ADC

	            // 锟斤拷锟斤拷OCB1校准系锟斤拷
	            Status = XRFdc_SetCalCoefficients(RFdcInstPtr, Tile, Block,
	                    XRFDC_CAL_BLOCK_OCB1, &OCB1_Calibration_Coefficients[2*Tile+Block]);

	            // 锟斤拷锟斤拷OCB2校准系锟斤拷
	            Status = XRFdc_SetCalCoefficients(RFdcInstPtr, Tile, Block,
	                    XRFDC_CAL_BLOCK_OCB2, &OCB2_Calibration_Coefficients[2*Tile+Block]);

	            // 锟斤拷锟斤拷GCB校准系锟斤拷
	            Status = XRFdc_SetCalCoefficients(RFdcInstPtr, Tile, Block,
	                    XRFDC_CAL_BLOCK_GCB, &GCB_Calibration_Coefficients[2*Tile+Block]);

	            // 锟斤拷锟斤拷TSCB校准系锟斤拷
	            Status = XRFdc_SetCalCoefficients(RFdcInstPtr, Tile, Block,
	                    XRFDC_CAL_BLOCK_TSCB, &TSCB_Calibration_Coefficients[2*Tile+Block]);

	            // 锟斤拷取锟斤拷OCB1系锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷证锟斤拷锟矫ｏ拷锟斤拷锟叫碉拷锟解，锟斤拷锟皆匡拷锟斤拷锟斤拷锟斤拷频锟斤拷锟角凤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟铰硷拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷谋锟叫Ｗ计碉拷省锟�
	            Status = XRFdc_GetCalCoefficients(RFdcInstPtr, Tile, Block, 0, &OCB1_Calibration_Coefficients[2*Tile+Block]);
	            Status = XRFdc_GetCalCoefficients(RFdcInstPtr, Tile, Block, 0, &OCB1_Calibration_Coefficients[2*Tile+Block]);
	            Status = XRFdc_GetCalCoefficients(RFdcInstPtr, Tile, Block, 0, &OCB1_Calibration_Coefficients[2*Tile+Block]);

	            // 锟斤拷取锟斤拷OCB2系锟斤拷锟斤拷锟斤拷
	            Status = XRFdc_GetCalCoefficients(RFdcInstPtr, Tile, Block, 1, &OCB2_Calibration_Coefficients[2*Tile+Block]);
	            Status = XRFdc_GetCalCoefficients(RFdcInstPtr, Tile, Block, 1, &OCB2_Calibration_Coefficients[2*Tile+Block]);
	            Status = XRFdc_GetCalCoefficients(RFdcInstPtr, Tile, Block, 1, &OCB2_Calibration_Coefficients[2*Tile+Block]);

	            // 锟斤拷取锟斤拷GCB系锟斤拷锟斤拷锟斤拷
	            Status = XRFdc_GetCalCoefficients(RFdcInstPtr, Tile, Block, 2, &GCB_Calibration_Coefficients[2*Tile+Block]);
	            Status = XRFdc_GetCalCoefficients(RFdcInstPtr, Tile, Block, 2, &GCB_Calibration_Coefficients[2*Tile+Block]);
	            Status = XRFdc_GetCalCoefficients(RFdcInstPtr, Tile, Block, 2, &GCB_Calibration_Coefficients[2*Tile+Block]);

	            // 锟斤拷取锟斤拷TSCB系锟斤拷锟斤拷锟斤拷
	            Status = XRFdc_GetCalCoefficients(RFdcInstPtr, Tile, Block, 3, &TSCB_Calibration_Coefficients[2*Tile+Block]);
	            Status = XRFdc_GetCalCoefficients(RFdcInstPtr, Tile, Block, 3, &TSCB_Calibration_Coefficients[2*Tile+Block]);
	            Status = XRFdc_GetCalCoefficients(RFdcInstPtr, Tile, Block, 3, &TSCB_Calibration_Coefficients[2*Tile+Block]);

	            // 锟斤拷锟叫Ｗ寄Ｊ窖★拷锟轿�0x01锟斤拷锟斤拷锟斤拷系锟斤拷锟斤拷锟角ｏ拷锟斤拷锟斤拷锟斤拷台校准锟斤拷锟叫ｏ拷
	            if(CAL_Mode_Sel == 0x01) {
	                Status = XRFdc_DisableCoefficientsOverride(RFdcInstPtr, Tile, Block, XRFDC_CAL_BLOCK_OCB1); // 锟斤拷锟斤拷OCB1锟斤拷锟斤拷
	                Status = XRFdc_DisableCoefficientsOverride(RFdcInstPtr, Tile, Block, XRFDC_CAL_BLOCK_OCB2); // 锟斤拷锟斤拷OCB2锟斤拷锟斤拷
	                Status = XRFdc_DisableCoefficientsOverride(RFdcInstPtr, Tile, Block, XRFDC_CAL_BLOCK_GCB); // 锟斤拷锟斤拷GCB锟斤拷锟斤拷
	                Status = XRFdc_DisableCoefficientsOverride(RFdcInstPtr, Tile, Block, XRFDC_CAL_BLOCK_TSCB); // 锟斤拷锟斤拷TSCB锟斤拷锟斤拷
	            }

	            // 锟斤拷锟斤拷锟斤拷sinc FIR锟剿诧拷锟斤拷锟斤拷锟斤拷锟矫ｏ拷
	            Status = XRFdc_SetInvSincFIR(RFdcInstPtr, Tile, Block, 0);

	            // 锟斤拷锟斤拷Nyquist锟斤拷锟斤拷为锟斤拷锟斤拷锟斤拷锟斤拷
	            Status = XRFdc_SetNyquistZone(RFdcInstPtr, RF_ADC, Tile, Block, NyquistZone_ODD);
	            Status = XRFdc_SetNyquistZone(RFdcInstPtr, RF_DAC, Tile, 2*Block, NyquistZone_ODD);

	            // 锟斤拷锟斤拷DAC锟斤拷锟斤拷锟斤拷剩锟阶拷偷锟斤拷硕锟斤拷锟斤拷锟绞硷拷锟斤拷
	            Status = XRFdc_SetDACVOP(RFdcInstPtr, Tile, 2*Block, 10000); // 锟斤拷锟斤拷DAC锟斤拷锟斤拷锟斤拷锟轿�10000
	            Status = XRFdc_SetDACVOP(RFdcInstPtr, Tile, 2*Block, 20000); // 锟斤拷锟斤拷DAC锟斤拷锟斤拷锟斤拷锟轿�20000
	            // Status = XRFdc_SetDACVOP(RFdcInstPtr, Tile, Block, 32000);
	            // Status = XRFdc_SetDACVOP(RFdcInstPtr, Tile, Block, 40000); // 锟斤拷锟斤拷涔︼拷锟�

	            // 锟斤拷锟斤拷ADC DSA锟斤拷锟斤拷锟街诧拷锟斤拷衰锟斤拷锟斤拷锟斤拷
	            XRFdc_DSA_Struc.DisableRTS = 1;  // 锟斤拷锟斤拷实时锟脚猴拷
	            XRFdc_DSA_Struc.Attenuation = 0; // 衰锟斤拷锟斤拷锟斤拷为0
	            Status = XRFdc_SetDSA(RFdcInstPtr, Tile, Block, &XRFdc_DSA_Struc); // 锟斤拷锟斤拷DSA
	            Status = XRFdc_GetDSA(RFdcInstPtr, Tile, Block, &XRFdc_DSA_Struc); // 锟斤拷取DSA锟斤拷锟斤拷锟斤拷锟斤拷证

	            // 锟斤拷锟斤拷ADC锟斤拷频锟斤拷锟斤拷锟斤拷
	            Status = XRFdc_GetMixerSettings(RFdcInstPtr, RF_ADC, Tile, Block, &ADC_Mixer_Settings);
	            ADC_Mixer_Settings.Freq = 000.0; // 锟斤拷锟矫伙拷频锟斤拷频锟斤拷为0
	            Status = XRFdc_SetMixerSettings(RFdcInstPtr, RF_ADC, Tile, Block, &ADC_Mixer_Settings);
	            Status = XRFdc_UpdateEvent(RFdcInstPtr, RF_ADC, Tile, Block, XRFDC_EVENT_MIXER); // 锟斤拷锟铰伙拷频锟斤拷锟铰硷拷
	            Status = XRFdc_GetMixerSettings(RFdcInstPtr, RF_ADC, Tile, Block, &ADC_Mixer_Settings); // 锟斤拷证锟斤拷锟斤拷

	            // 锟斤拷锟斤拷DAC锟斤拷频锟斤拷锟斤拷锟斤拷
	            Status = XRFdc_GetMixerSettings(RFdcInstPtr, RF_DAC, Tile, 2*Block, &DAC_Mixer_Settings);
	            DAC_Mixer_Settings.Freq = 0000.0; // 锟斤拷锟矫伙拷频锟斤拷频锟斤拷为0
	            Status = XRFdc_SetMixerSettings(RFdcInstPtr, RF_DAC, Tile, 2*Block, &DAC_Mixer_Settings);
	            Status = XRFdc_UpdateEvent(RFdcInstPtr, RF_DAC, Tile, 2*Block, XRFDC_EVENT_MIXER); // 锟斤拷锟铰伙拷频锟斤拷锟铰硷拷
	            Status = XRFdc_GetMixerSettings(RFdcInstPtr, RF_DAC, Tile, 2*Block, &DAC_Mixer_Settings); // 锟斤拷证锟斤拷锟斤拷

	            // 锟斤拷锟斤拷Nyquist锟斤拷锟斤拷为锟斤拷锟斤拷锟斤拷锟斤拷锟劫达拷确锟较ｏ拷
	            Status = XRFdc_SetNyquistZone(RFdcInstPtr, RF_ADC, Tile, Block, NyquistZone_ODD);
	            Status = XRFdc_SetNyquistZone(RFdcInstPtr, RF_DAC, Tile, 2*Block, NyquistZone_ODD);

	            // 锟斤拷取锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷路锟斤拷模式
	            Status = XRFdc_GetDataPathMode(RFdcInstPtr, Tile, Block, &DataPath_Mode);
	            Status = XRFdc_SetDataPathMode(RFdcInstPtr, Tile, Block, DataPath_Mode);
	            Status = XRFdc_GetDataPathMode(RFdcInstPtr, Tile, Block, &DataPath_Mode); // 锟斤拷证锟斤拷锟斤拷

	            // 注锟酵碉拷锟侥达拷锟诫：锟截闭猴拷锟斤拷锟斤拷ADC/DAC
	            // Status = XRFdc_Shutdown(RFdcInstPtr, RF_ADC, Tile); // 锟截憋拷ADC
	            // Status = XRFdc_StartUp(RFdcInstPtr, RF_ADC, Tile); // 锟斤拷锟斤拷ADC
	            // Status = XRFdc_Shutdown(RFdcInstPtr, RF_DAC, Tile); // 锟截憋拷DAC
	            // Status = XRFdc_StartUp(RFdcInstPtr, RF_DAC, Tile); // 锟斤拷锟斤拷DAC

	            // 注锟酵碉拷锟侥达拷锟诫：锟斤拷锟斤拷锟斤拷sinc FIR锟剿诧拷锟斤拷锟侥讹拷锟斤拷锟斤拷锟�
	            // Status = XRFdc_SetInvSincFIR(RFdcInstPtr, Tile, Block, 0);
	            // Status = XRFdc_SetInvSincFIR(RFdcInstPtr, Tile, Block, 1);
	            // Status = XRFdc_SetInvSincFIR(RFdcInstPtr, Tile, Block, 2);
	        }
	    }
	}

	MTS_SYNC_Function(RFdcDeviceId);


	return XRFDC_SUCCESS;
}

/****************************************************************************/
/*
*
* This function compares the two Fabric Rate variables and return 0 if
* same and returns 1 if not same.
*
* @param	SetFabricRate Fabric Rate value set.
* @param	GetFabricRate Fabric Rate value get.
*
* @return
*			- 0 if both structures are same.
*			- 1 if both structures are not same.
*
* @note		None
*
*****************************************************************************/
static int CompareFabricRate(u32 SetFabricRate, u32 GetFabricRate)
{
	if (SetFabricRate == GetFabricRate)
		return 0;
	else
		return 1;
}

static unsigned Hmc7044_ReadRegister(unsigned reg)
{
    DevFpgaWrite(HMC7044_RD_ADDR, 0x8000U | reg);
    usleep(20);
    DevFpgaWrite(HMC7044_RD_ENABLE_ADDR, 1U);
    usleep(20);
    DevFpgaWrite(HMC7044_RD_ENABLE_ADDR, 0U);
    usleep(2000);
    return DevFpgaRead(HMC7044_BRAM_RD_ADDR) & 0xFFU;
}

int main()
{
    init_platform();
  int Status;
  int  hmc_rd_data =0;


    cleanup_platform();
    gpio_initial();
    bus_test_Data=DevFpgaRead(BUS_TEST_ADDR);

    xil_printf("Current SoftWare Version is  is %x%x year锟斤拷 %x month 锟斤拷 %x day \r\n",((bus_test_Data&0xff000000)>>24),((bus_test_Data&0xff0000)>>16),((bus_test_Data&0x0000ff00)>>8),(bus_test_Data&0xff));



//
//#define  BUS_TEST_ADDR          	 (0X00)
//#define  HMC7044_CONFIG_ADDR		 (0x00)
//#define  HMC7044_RECONFIG_ADDR		 (0x01)
//#define  HMC7044_RECONFIG_DATA_ADDR	 (0x02)
//#define  Sys_Rst_Addr			 	 (0x03)
//#define  DAC_ENABLE_ADDR	 	 	 (0x04)
//#define  HMC7044_SYNC_REQ_ADDR	 	 (0x06)
//#define  HMC7044_RD_ADDR	 		 (0x07)
//#define  DDS_SEL_ADDR		 		 (0x09)
//#define  HMC7044_RD_ENABLE_ADDR	 	 (0x0A)
//



    DevFpgaWrite(HMC7044_CONFIG_ADDR,0x01);
    usleep(2);
    DevFpgaWrite(HMC7044_CONFIG_ADDR,0x00);
    usleep(2);






    sleep(3);

//    DevFpgaWrite(HMC7044_SYNC_REQ_ADDR,0x01);
//    usleep(20);
//    DevFpgaWrite(HMC7044_SYNC_REQ_ADDR,0x00);
//    usleep(20);
    DevFpgaWrite(HMC7044_RD_ADDR,0x807c);
    usleep(20);
    DevFpgaWrite(HMC7044_RD_ENABLE_ADDR,0x01);///
    usleep(20);
    DevFpgaWrite(HMC7044_RD_ENABLE_ADDR,0x00);
    usleep(2000);


    hmc_rd_data=DevFpgaRead(HMC7044_BRAM_RD_ADDR);

    printf("Current HMC7044 PLL1 Status is %x\r\n",hmc_rd_data);


    DevFpgaWrite(HMC7044_RD_ADDR,0x807D);
    usleep(20);
    DevFpgaWrite(HMC7044_RD_ENABLE_ADDR,0x01);///
    usleep(20);
    DevFpgaWrite(HMC7044_RD_ENABLE_ADDR,0x00);
    usleep(2000);


    hmc_rd_data=DevFpgaRead(HMC7044_BRAM_RD_ADDR);


    printf("Current HMC7044 PLL Status is %x\r\n",hmc_rd_data);

    /* CH10 drives DAC_CLK2; CH12 drives the PL system clock. */
    {
        unsigned pll1 = Hmc7044_ReadRegister(0x007CU);
        unsigned ch10 = Hmc7044_ReadRegister(0x012DU);
        unsigned ch12 = Hmc7044_ReadRegister(0x0141U);
        printf("HMC7044 PLL1=0x%x PLL1_locked=%u PLL1+PLL2_locked=%u\r\n",
               pll1, (pll1 >> 5) & 1U, (hmc_rd_data >> 3) & 1U);
        printf("HMC7044 CH10 divider=0x%x CH12 divider=0x%x (expect 0x08)\r\n",
               ch10, ch12);
        if (((pll1 & 0x20U) == 0U) || ((hmc_rd_data & 0x08U) == 0U) ||
            ch10 != 0x08U || ch12 != 0x08U) {
            printf("ERROR: HMC7044 clock plan is not locked at /8\r\n");
            return XRFDC_FAILURE;
        }
    }

	//enable dac rf  trace
	DevFpgaWrite(DAC_RF_OPEN_ADDR,0x01);
	DevFpgaWrite(DAC_RF_EN_ADDR,0x00);
	DevFpgaWrite(DAC_DATA_SEL_ADDR,0x00);/////DAC  SEL DDS data

    printf("RFdc config begin----------------\r\n");
    	/*
    	 * Run the RFdc fabric rate example, specify the Device ID that is
    	 * generated in xparameters.h.
    	 */
    sleep(4);
        DevFpgaWrite(HMC7044_SYNC_REQ_ADDR,0x01);
    Status = metal_log_init();
    if (Status != XST_SUCCESS) {
    		printf(" metal_log_init failed\r\n");
    		return XRFDC_FAILURE;
    }
    /* RFdc Nyquist-2 / MTS / cal live in helloworld.c */
    Status = rf_Init();
    if (Status != XST_SUCCESS) {
    		printf(" RFSOC  config failed\r\n");
    		return XRFDC_FAILURE;
    }


    printf("Successfully run RFSOC  config  \r\n");

    {
        unsigned i, mon_status = 0;
        for (i = 0; i < 50U; i++) {
            mon_status = DevFpgaRead(RFDC_CLOCK_STATUS_ADDR);
            if ((mon_status & 0x7U) == 0x7U)
                break;
            usleep(20000);
        }
        printf("RFDC clock ADC tile=%u Hz, DAC tile=%u Hz, PL=%u Hz, status=0x%x\r\n",
               DevFpgaRead(RFDC_ADC_TILE_CLOCK_HZ_ADDR),
               DevFpgaRead(RFDC_DAC_TILE_CLOCK_HZ_ADDR),
               DevFpgaRead(RFDC_PL_CLOCK_HZ_ADDR), mon_status);
        printf("Clock status: valid=0x%x good=0x%x missing=0x%x ref_lock=%u seq=%u\r\n",
               mon_status & 7U, (mon_status >> 3) & 7U,
               (mon_status >> 6) & 7U, (mon_status >> 9) & 1U,
               mon_status >> 16);
        if ((mon_status & 0x3FU) != 0x3FU)
            printf("WARNING: RFDC clock monitor did not confirm three 300 MHz clocks\r\n");
    }

  while(1)
  {

  }

    return 0;
}

