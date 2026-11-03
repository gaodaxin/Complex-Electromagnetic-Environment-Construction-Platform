//MIO控制LED亮灭
#include "mio_gpio.h" //包含 sleep()函数


//宏定义 GPIO_DEVICE_ID，三个LED灯引脚
#define GPIO_DEVICE_ID XPAR_XGPIOPS_0_DEVICE_ID//连接到 MIO 的 LED
#define MIOLED0 25 //连接到 MIO7
XGpioPs Gpio; // GPIO 设备的驱动程序实例



int gpio_initial()
{
int Status;
XGpioPs_Config *ConfigPtr;//配置寄存器地址
print("MIO Test! \n\r");
ConfigPtr = XGpioPs_LookupConfig(GPIO_DEVICE_ID);//寻找GPIO的器件地址
Status = XGpioPs_CfgInitialize(&Gpio, ConfigPtr, ConfigPtr->BaseAddr);//将配置寄存器初始化

if (Status != XST_SUCCESS){
return XST_FAILURE;
}
//设置指定引脚的方向： 0 输入， 1 输出（控制DIRM,OEN）
XGpioPs_SetDirectionPin(&Gpio, MIOLED0, 1);

//使能指定引脚输出： 0 禁止输出使能， 1 使能输出
XGpioPs_SetOutputEnablePin(&Gpio, MIOLED0, 1);




//while (1) {
XGpioPs_WritePin(&Gpio, MIOLED0, 0x1); //向指定引脚写入数据： 0 或 1
//XGpioPs_WritePin(&Gpio, MIOLED1, 0x0);
//XGpioPs_WritePin(&Gpio, MIOLED2, 0x0);
//sleep(1); //延时 1 秒
//XGpioPs_WritePin(&Gpio, MIOLED0, 0x1);
//XGpioPs_WritePin(&Gpio, MIOLED1, 0x1);
//XGpioPs_WritePin(&Gpio, MIOLED2, 0x1);
//sleep(1);
//}
return XST_SUCCESS;
}
