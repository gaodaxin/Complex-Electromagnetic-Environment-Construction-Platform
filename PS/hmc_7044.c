///*
// * hmc_7044.c
// *
// *  Created on: 2019年10月28日
// *      Author: yangjian
// */
//#include"hmc_7044.h"
//
//
//
//
//uint8_t      Coarse_Delay_Array[5]={0x00,0x00,0x00,0x00,0x00};
////***********************************************************
////hmc config
////***********************************************************
//
//void HMC7044_Sysref_SW(uint32_t uiMode)
//{
//	uint8_t tmp_data =0;
//
//	if(uiMode == 0)//sync disable
//		{
//		DevFpgaWrite(0x51,0x80C8);//////////////  Channel 0
//		DevFpgaWrite(0x50,0x0001);//////////////
//		DevFpgaWrite(0x50,0x0000);//////////////
//		mdelay(1);
//		tmp_data = DevFpgaRead(0x51);
//		tmp_data = tmp_data&0xbf;
//
//		DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x00c8);
//		DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//		DevFpgaWrite(0x51,0x80D2);/////////////////Channel 1
//		DevFpgaWrite(0x50,0x0001);//////////////
//		DevFpgaWrite(0x50,0x0000);//////////////
//		mdelay(1);
//		tmp_data = DevFpgaRead(0x51);
//		tmp_data = tmp_data&0xbf;
//		DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x00D2);
//		DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//
//		DevFpgaWrite(0x51,0x80DC);/////////////////Channel 2
//		DevFpgaWrite(0x50,0x0001);//////////////
//		DevFpgaWrite(0x50,0x0000);//////////////
//		mdelay(1);
//		tmp_data = DevFpgaRead(0x51);
//		tmp_data = tmp_data&0xbf;
//		DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x00DC);
//		DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//
//		DevFpgaWrite(0x51,0x80E6);/////////////////Channel 3
//		DevFpgaWrite(0x50,0x0001);//////////////
//		DevFpgaWrite(0x50,0x0000);//////////////
//		mdelay(1);
//		tmp_data = DevFpgaRead(0x51);
//		tmp_data = tmp_data&0xbf;
//		DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x00E6);
//		DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//
//		DevFpgaWrite(0x51,0x80F0);/////////////////Channel 4
//		DevFpgaWrite(0x50,0x0001);//////////////
//		DevFpgaWrite(0x50,0x0000);//////////////
//		mdelay(1);
//		tmp_data = DevFpgaRead(0x51);
//		tmp_data = tmp_data&0xbf;
//		DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x00F0);
//		DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//
//
//		DevFpgaWrite(0x51,0x80FA);/////////////////Channel 5
//		DevFpgaWrite(0x50,0x0001);//////////////
//		DevFpgaWrite(0x50,0x0000);//////////////
//		mdelay(1);
//		tmp_data = DevFpgaRead(0x51);
//		tmp_data = tmp_data&0xbf;
//		DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x00FA);
//		DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//		DevFpgaWrite(0x51,0x8104);/////////////////Channel 6
//		DevFpgaWrite(0x50,0x0001);//////////////
//		DevFpgaWrite(0x50,0x0000);//////////////
//		mdelay(1);
//		tmp_data = DevFpgaRead(0x51);
//		tmp_data = tmp_data&0xbf;
//		DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x0104);
//		DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//		DevFpgaWrite(0x51,0x810E);/////////////////Channel 7
//		DevFpgaWrite(0x50,0x0001);//////////////
//		DevFpgaWrite(0x50,0x0000);//////////////
//		mdelay(1);
//		tmp_data = DevFpgaRead(0x51);
//		tmp_data = tmp_data&0xbf;
//		DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x010E);
//		DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//
//		DevFpgaWrite(0x51,0x8118);/////////////////Channel 8
//		DevFpgaWrite(0x50,0x0001);//////////////
//		DevFpgaWrite(0x50,0x0000);//////////////
//		mdelay(1);
//		tmp_data = DevFpgaRead(0x51);
//		tmp_data = tmp_data&0xbf;
//		DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x0118);
//		DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//
//		DevFpgaWrite(0x51,0x8122);/////////////////Channel 9
//		DevFpgaWrite(0x50,0x0001);//////////////
//		DevFpgaWrite(0x50,0x0000);//////////////
//		mdelay(1);
//		tmp_data = DevFpgaRead(0x51);
//		tmp_data = tmp_data&0xbf;
//		DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x0122);
//		DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//
//		DevFpgaWrite(0x51,0x812C);/////////////////Channel 10
//		DevFpgaWrite(0x50,0x0001);//////////////
//		DevFpgaWrite(0x50,0x0000);//////////////
//		mdelay(1);
//		tmp_data = DevFpgaRead(0x51);
//		tmp_data = tmp_data&0xbf;
//		DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x012C);
//		DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//
//		DevFpgaWrite(0x51,0x8136);/////////////////Channel 11
//		DevFpgaWrite(0x50,0x0001);//////////////
//		DevFpgaWrite(0x50,0x0000);//////////////
//		mdelay(1);
//		tmp_data = DevFpgaRead(0x51);
//		tmp_data = tmp_data&0xbf;
//		DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x0136);
//		DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//		DevFpgaWrite(0x51,0x8140);/////////////////Channel 12
//		DevFpgaWrite(0x50,0x0001);//////////////
//		DevFpgaWrite(0x50,0x0000);//////////////
//		mdelay(1);
//		tmp_data = DevFpgaRead(0x51);
//		tmp_data = tmp_data&0xbf;
//		DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x0140);
//		DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//		DevFpgaWrite(0x51,0x814A);/////////////////Channel 13
//		DevFpgaWrite(0x50,0x0001);//////////////
//		DevFpgaWrite(0x50,0x0000);//////////////
//		mdelay(1);
//		tmp_data = DevFpgaRead(0x51);
//		tmp_data = tmp_data&0xbf;
//		DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x014A);
//		DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//
//		}
//		else if(uiMode == 1)//open sysref
//		{
//			DevFpgaWrite(0x51,0x80C8);//////////////  Channel 0
//			DevFpgaWrite(0x50,0x0001);//////////////
//			DevFpgaWrite(0x50,0x0000);//////////////
//			mdelay(1);
//			tmp_data = DevFpgaRead(0x51);
//			tmp_data = tmp_data|0x40;
//
//			DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x00c8);
//			DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//			DevFpgaWrite(0x51,0x80D2);/////////////////Channel 1
//			DevFpgaWrite(0x50,0x0001);//////////////
//			DevFpgaWrite(0x50,0x0000);//////////////
//			mdelay(1);
//			tmp_data = DevFpgaRead(0x51);
//			tmp_data = tmp_data|0x40;
//			DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x00D2);
//			DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//
//			DevFpgaWrite(0x51,0x80DC);/////////////////Channel 2
//			DevFpgaWrite(0x50,0x0001);//////////////
//			DevFpgaWrite(0x50,0x0000);//////////////
//			mdelay(1);
//			tmp_data = DevFpgaRead(0x51);
//			tmp_data = tmp_data|0x40;
//			DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x00DC);
//			DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//
//			DevFpgaWrite(0x51,0x80E6);/////////////////Channel 3
//			DevFpgaWrite(0x50,0x0001);//////////////
//			DevFpgaWrite(0x50,0x0000);//////////////
//			mdelay(1);
//			tmp_data = DevFpgaRead(0x51);
//			tmp_data = tmp_data|0x40;
//			DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x00E6);
//			DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//
//			DevFpgaWrite(0x51,0x80F0);/////////////////Channel 4
//			DevFpgaWrite(0x50,0x0001);//////////////
//			DevFpgaWrite(0x50,0x0000);//////////////
//			mdelay(1);
//			tmp_data = DevFpgaRead(0x51);
//			tmp_data = tmp_data|0x40;
//			DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x00F0);
//			DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//
//
//			DevFpgaWrite(0x51,0x80FA);/////////////////Channel 5
//			DevFpgaWrite(0x50,0x0001);//////////////
//			DevFpgaWrite(0x50,0x0000);//////////////
//			mdelay(1);
//			tmp_data = DevFpgaRead(0x51);
//			tmp_data = tmp_data|0x40;
//			DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x00FA);
//			DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//			DevFpgaWrite(0x51,0x8104);/////////////////Channel 6
//			DevFpgaWrite(0x50,0x0001);//////////////
//			DevFpgaWrite(0x50,0x0000);//////////////
//			mdelay(1);
//			tmp_data = DevFpgaRead(0x51);
//			tmp_data = tmp_data|0x40;
//			DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x0104);
//			DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//			DevFpgaWrite(0x51,0x810E);/////////////////Channel 7
//			DevFpgaWrite(0x50,0x0001);//////////////
//			DevFpgaWrite(0x50,0x0000);//////////////
//			mdelay(1);
//			tmp_data = DevFpgaRead(0x51);
//			tmp_data = tmp_data|0x40;
//			DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x010E);
//			DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//
//			DevFpgaWrite(0x51,0x8118);/////////////////Channel 8
//			DevFpgaWrite(0x50,0x0001);//////////////
//			DevFpgaWrite(0x50,0x0000);//////////////
//			mdelay(1);
//			tmp_data = DevFpgaRead(0x51);
//			tmp_data = tmp_data|0x40;
//			DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x0118);
//			DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//
//			DevFpgaWrite(0x51,0x8122);/////////////////Channel 9
//			DevFpgaWrite(0x50,0x0001);//////////////
//			DevFpgaWrite(0x50,0x0000);//////////////
//			mdelay(1);
//			tmp_data = DevFpgaRead(0x51);
//			tmp_data = tmp_data|0x40;
//			DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x0122);
//			DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//
//			DevFpgaWrite(0x51,0x812C);/////////////////Channel 10
//			DevFpgaWrite(0x50,0x0001);//////////////
//			DevFpgaWrite(0x50,0x0000);//////////////
//			mdelay(1);
//			tmp_data = DevFpgaRead(0x51);
//			tmp_data = tmp_data|0x40;
//			DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x012C);
//			DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//
//			DevFpgaWrite(0x51,0x8136);/////////////////Channel 11
//			DevFpgaWrite(0x50,0x0001);//////////////
//			DevFpgaWrite(0x50,0x0000);//////////////
//			mdelay(1);
//			tmp_data = DevFpgaRead(0x51);
//			tmp_data = tmp_data|0x40;
//			DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x0136);
//			DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//			DevFpgaWrite(0x51,0x8140);/////////////////Channel 12
//			DevFpgaWrite(0x50,0x0001);//////////////
//			DevFpgaWrite(0x50,0x0000);//////////////
//			mdelay(1);
//			tmp_data = DevFpgaRead(0x51);
//			tmp_data = tmp_data|0x40;
//			DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x0140);
//			DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//			DevFpgaWrite(0x51,0x814A);/////////////////Channel 13
//			DevFpgaWrite(0x50,0x0001);//////////////
//			DevFpgaWrite(0x50,0x0000);//////////////
//			mdelay(1);
//			tmp_data = DevFpgaRead(0x51);
//			tmp_data = tmp_data|0x40;
//			DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,0x014A);
//			DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,tmp_data);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//			DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//
//
//		}
//		else
//		{
//			;
//		}
//
//
//
//}
//
//
//
////*************************************************
////sync 请求函数
////*************************************************
//
//void  Hmc_Sync_Req( hmc_SysrefReqMode_t  mode)
//
//{
//
//	if (mode == SYSREF_CONT_ON)
//	{
//		DevFpgaWrite(0xaa,0x01);
//		DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,PULSE_REQ_ADDR);
//		DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,0x0007);
//
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//		mdelay(1);
//
//		DevFpgaWrite(PULSE_SYNC_ADDR,0x01);
//	}
//	else if (mode == SYSREF_CONT_OFF)
//		{
//
//
//		DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,PULSE_REQ_ADDR);
//		DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,0x0000);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//		DevFpgaWrite(PULSE_SYNC_ADDR,0x00);///disable sync function
//
//
//
//		mdelay(1);
//
//
//		}
//	else if (mode == SYSREF_PULSE) {
//
//
//		DevFpgaWrite(HMC_CONFIG_DATA_L_ADDR,PULSE_REQ_ADDR);
//		DevFpgaWrite(HMC_CONFIG_DATA_H_ADDR,0x0003);/////generate 16 pulse
//
//
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x01);
//		DevFpgaWrite(PS_WR_START_EN_ADDR,0x00);
//
//
//
//
//		mdelay(1);
//
//		DevFpgaWrite(PULSE_SYNC_ADDR,0x01);
//		mdelay(1);
//		DevFpgaWrite(PULSE_SYNC_ADDR,0x00);
//		mdelay(1);
//
//
//
//	}
//
//
//
//
//}
//
//
//
////*****************************************************************
////hmc_delay_config
////
////*****************************************************************
//void   hmc_delay_config(void)
//{
//   uint8_t i=0;
//   for(i=0;i<5;i++)
//   {
//
//   		DevFpgaWrite(0x04+i,Coarse_Delay_Array[i]&0x1f);
//
//   }
//
//}
