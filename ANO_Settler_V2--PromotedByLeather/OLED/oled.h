/*
	程序使用说明：
	1.本程序仅支持oled模块在SPI下的控制即4线通信，并不支持其他的
工作模式；oled的常用控制命令见文件夹里图片
	2.在修改区中需根据实际情况修改相应电路的引脚定义，默认情况下:
		时钟线：PA0  数据线：PA1  数据/命令选择线：PA2  复位线：PA3
		时钟和端口的配置在修改区中修改，默认下都配置为推挽输出；
	3.接口说明，在使用OLED前必须初始化OLED,函数为：OLED_Init();
有时候会用到清屏，故提供如下接口：
			Clear_Color; //将屏幕清为彩色； 					
			Clear_Black; //将屏幕清为全暗；
显示汉字时，可用函数： OLED_Show_FONT_16x16(unsigned char X,unsigned char Y,unsigned char Num);
该函数变量说明：X表示第几行(OLED在16*16汉字显示下可以显示4行，8列)，Y为第几列，Num为字库中第几
个汉字，这三个变量的下标都由1开始，符合应用需要，若想查看字库，则开"oled.c"中打开包含的头文件：
"oled_word_library.h" 在字库中借助取字模软件得到相应的汉字，存放在相应模式的库中直接使用；

以下函数接口的使用都需要在"oled_word_library.h"中打开相应的宏定义才能使用；
	a. 显示8*16大小字符的接口：
			OLED_Show_FONT_8x16(unsigned char x , unsigned char y , unsigned char Data[]);
此接口的变量说明：x: 行(1~4)   y: 列(1~16)   Data[]为要显示的字符串；最大显示为4行16列;
	b. 显示6*8大小字符接口：
			void OLED_Show_FONT_6x8(unsigned char x , unsigned char y , unsigned char Data[]);
此接口的变量与上一函数作用一样，不过范围可达到：8行21列；
	c. 显示图片接口：
				void OLED_Show_PICTURE_128x64(unsigned char x0 , unsigned char y0,
								              unsigned char x1 , unsigned char y1,unsigned int Picture_Index);
此函数变量说明：
		( x0 , y0 ) 初始坐标，x0: 起始行(1~8)   y0: 起始列(0~128);
		( x1 , y1 ) 终点坐标，x1: 终点行(1~8)   y1: 终点列(0~128);
		自行根据规范使用，不按规范使用时没有提供错误处理功能；

*/
#ifndef __OLED_H__
#define __OLED_H__

/****************************************************************************/
/////////////////////////////////////////////////////////////////////////////
////		                        修改区                 			  ///////
/////////////////////////////////////////////////////////////////////////////
/*------------OLED引脚定义------------*/
		//OLED时钟线；
		#define OLED_CLK  PBout(5)   //D0
		
		//OLED数据线；
		#define OLED_DAT PBout(4)		//D1
		
		//OLED数据/命令选择线；
		#define OLED_DC	 PAout(15)   //DC
		/////
		//数据，命令选择：
		#define OLED_COMMAND 0
		#define OLED_DATA    1
		
		//OLED复位线；
		#define OLED_RST PBout(3)	//res


//开启相应端口时钟并配置端口
//#define OLED_SET_GPIO { RCC->APB2ENR|=1<<2;GPIOA->CRL&=0xffff0000;GPIOA->CRL|=0xffff3333; GPIOA->ODR|=0x0c0; }///
					   //使能时钟；         预清0					推挽输出；           DC.RST置1，DAT.CMD置0；

/* 如果要修改的地方较多，可以采用以下定义，不同在于'\'做连接，避免宏定义过长；
#define OLED_SET_GPIO { \
						 RCC->APB2ENR|=1<<2; \
						 GPIOA->CRL&=0xffff0000; \
						 GPIOA->CRL|=0xffff3333; \
						 GPIOA->ODR|=0x0c0; \
 					  }
*/

/*-------------------------------------------以上根据实际自行更改----------------------------------------------------*/


//特殊定义,不可随便更改；
#define OLED_PAGE 8 //oled显存为8页，每页128个字节；
#define OLED_X 128	//oled长；
#define OLED_Y 64	//oled宽；
#define Clear_Color OLED_Screen(0xff)	//清屏，有色；
#define Clear_Black OLED_Screen(0x00)	//清屏，无色；



//OLED初始化函数；
void OLED_Init(void);

//写入命令函数；
void OLED_WriteCommand(unsigned char Cmd);

//写入数据函数；
void OLED_WriteByte(unsigned char Data);

//全屏操作函数；
void OLED_Screen(unsigned char Val);

//坐标设置函数；
void OLED_SetPosition(unsigned char x , unsigned char y);

//16*16 字体模式显示函数；
void OLED_Show_FONT_16x16(unsigned char x , unsigned char y , unsigned char Num);

void OledShowData_float(float axis,unsigned char HANG,unsigned char LIE);
#if FONT_8x16_MODE //若定义了该宏，则表示8*16 ASCII码标准字体模式开启；
	//8*16 字体模式显示函数；
	void OLED_Show_FONT_8x16(unsigned char x , unsigned char y , unsigned char Data[]);
////////////////////////////////////////////////////////////////////////////////////////
#endif


#if FONT_6x8_MODE //若定义了该宏，则表示6*8 ASCII码标准字体模式开启；
	//6*8 字体模式显示函数；
	 void OLED_Show_FONT_6x8(unsigned char x , unsigned char y , unsigned char Data[]);
////////////////////////////////////////////////////////////////////////////////////////
#endif


#if PICTURE_MODE  //若定义了该宏，则表示图片显示模式开启；
	//最大128*64 图片模式显示函数；
	void OLED_Show_PICTURE_128x64(unsigned char x0 , unsigned char y0,
								  unsigned char x1 , unsigned char y1,unsigned int Picture_Index);
//////////////////////////////////////////////////////////////////////////////////////////////////
#endif
#endif










