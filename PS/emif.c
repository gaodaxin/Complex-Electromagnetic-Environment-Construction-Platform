#include"emif.h"
void  DevFpgaWrite(uint16_t usAddr,uint32_t Data)
{
	 Xil_Out32(BRAM_BASE_ADDR+usAddr*4,Data);

}


uint32_t DevFpgaRead(uint16_t usAddr)
{
	 return  Xil_In32(BRAM_BASE_ADDR+usAddr*4);

}
