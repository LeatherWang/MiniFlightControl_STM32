
#include "oled_word_library.h" //OLED字库；
#include "oled.h"
#include "bsp_SysTick.h"
#include "stm32f10x.h"
#include "gpio_leather.h"

//宏观上初始化OLED，其中调用到的函数在后面加以定义;
void OLED_Init(void)
{ 
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	

	  OLED_CLK=1;	 //拉高时钟线；
    OLED_RST=0;	 //复位oled;
    Delay_ms(50);
    OLED_RST=1;      //停止复位；   
    OLED_WriteCommand(0xae);//--turn off oled panel
    OLED_WriteCommand(0x00);//---set low column address
    OLED_WriteCommand(0x10);//---set high column address
    OLED_WriteCommand(0x40);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    OLED_WriteCommand(0x81);//--set contrast control register
    OLED_WriteCommand(0xcf); // Set SEG Output Current Brightness
    OLED_WriteCommand(0xa1);//--Set SEG/Column Mapping     
    OLED_WriteCommand(0xc8);//Set COM/Row Scan Direction  
    OLED_WriteCommand(0xa6);//--set normal display
    OLED_WriteCommand(0xa8);//--set multiplex ratio(1 to 64)
    OLED_WriteCommand(0x3f);//--1/64 duty
    OLED_WriteCommand(0xd3);//-set display offset    Shift Mapping RAM Counter (0x00~0x3F)
    OLED_WriteCommand(0x00);//-not offset
    OLED_WriteCommand(0xd5);//--set display clock divide ratio/oscillator frequency
    OLED_WriteCommand(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
    OLED_WriteCommand(0xd9);//--set pre-charge period
    OLED_WriteCommand(0xf1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    OLED_WriteCommand(0xda);//--set com pins hardware configuration
    OLED_WriteCommand(0x12);
    OLED_WriteCommand(0xdb);//--set vcomh
    OLED_WriteCommand(0x40);//Set VCOM Deselect Level
    OLED_WriteCommand(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02)
    OLED_WriteCommand(0x02);//
    OLED_WriteCommand(0x8d);//--set Charge Pump enable/disable
    OLED_WriteCommand(0x14);//--set(0x10) disable
    OLED_WriteCommand(0xa4);// Disable Entire Display On (0xa4/0xa5)
    OLED_WriteCommand(0xa6);// Disable Inverse Display On (0xa6/a7) 
    OLED_WriteCommand(0xaf);//--turn on oled panel
    //Clear_Color;  //初始清屏,有色；
	Clear_Black; //初始清屏,无色
	OLED_SetPosition(0,0); //设置起始点坐标；
}

void OLED_WriteCommand(unsigned char Cmd)
{
	unsigned char i,Val;
    OLED_DC = OLED_COMMAND;	//数据/命令选择线选命令；
    for(i=0;i<8;i++) //发送一个字节,发送方向从高位开始； 
    { 
        OLED_CLK=0;  //拉低时钟线     
        Val = Cmd & 0x80;
        if (Val == 0)
        {
            OLED_DAT = 0;
        }
        else
        {
            OLED_DAT = 1;
        }
        OLED_CLK=1;	//拉高时钟线，产生上升沿，一位数据被传送；
        Cmd<<=1;;        
    }     
}

void OLED_WriteByte(unsigned char Data)
{
	unsigned char i,Val;
    OLED_DC = OLED_DATA;	//数据/命令选择线选数据；
    for(i=0;i<8;i++) //发送一个字节,发送方向从高位开始； 
    { 
        OLED_CLK=0;  //拉低时钟线     
        Val = Data & 0x80;
        if (Val == 0)
        {
            OLED_DAT = 0;
        }
        else
        {
            OLED_DAT = 1;
        }
        OLED_CLK=1;	//拉高时钟线，产生上升沿，一位数据被传送；
        Data<<=1;;        
    }     
}

//当Val = 0xff清屏，提供变量Val加大功能；
void OLED_Screen(unsigned char Val)
{
	unsigned char x,y; //x为页变量，最大为OLED_PAGE；
	for(x=0;x<OLED_PAGE;x++)
	{
		OLED_WriteCommand(0xb0+x);	//命令作用见文件夹中的图示；
		OLED_WriteCommand(0x01);
		OLED_WriteCommand(0x10);
		for(y=0;y<OLED_X;y++) { OLED_WriteByte(Val); }
	}
}

void OLED_SetPosition(unsigned char x , unsigned char y)  //y为页；
{
	OLED_WriteCommand(0xb0+y);
    OLED_WriteCommand(((x&0xf0)>>4)|0x10);
    OLED_WriteCommand((x&0x0f)|0x01); 
}

/*
	16*16模式一个汉字总共需要显示256个点,256/8=32,即必须一个汉字由32个字节确定;
	函数原理：当显示模式采用16*16时；oled(128*64)能容纳4行8列，即最多32个汉字；每行8个汉字；
oled没行必须由两个页存储，因为一个页只有128个字节，而每个汉字32个字节，故一个页容纳4个汉字；
偏移量对应字库里面的第几个汉字，下标为0的那个汉字即第一个汉字无偏移量，第二个汉字要偏移第一
个汉字的容量即偏移量，16*16模式中偏移量为32个字节；
	4行8列
*/
//X为行，Y为列，Num为字库中的第几个汉字，下标从1开始，更加符合使用规范；
void OLED_Show_FONT_16x16(unsigned char x , unsigned char y , unsigned char Num) //N为偏移量；
{
	unsigned char i;
    unsigned int adder;  //偏移量，作用见下一个函数说明；	
	{
		Num--;
		x--;
		y--;
		x *= 2;
		y *= 16;  
	}//行列到点的转换；
	adder = 32*Num;    
    OLED_SetPosition(y , x); //一个汉字由两个页决定，第一页； 
    for(i = 0;i < 16;i++)              
    {
        OLED_WriteByte(FONT_16x16[adder]); //写入字节；   
        adder += 1;
    }      
    OLED_SetPosition(y,x + 1); //第二页；   
    for(i = 0;i < 16;i++)       
    {
        OLED_WriteByte(FONT_16x16[adder]);
        adder += 1;
    }           	
}


#if FONT_8x16_MODE //若定义了该宏，则表示6*8 ASCII码标准字体模式开启；
//8*16 字体模式显示函数；
/*
	 x: 行，y: 列
	128*64oled 在8*16模式下可以最多显示64个字符，每个字符必须占用两个页中的内容；
故4行，64/4 = 16，即每列16个字符；
	4行16列
*/
void OLED_Show_FONT_8x16(unsigned char x , unsigned char y , unsigned char Data[])
{
	unsigned char ascii_num = 0, //字符转换成ASCII码后在字库中码表里的具体位置；
				  count = 0, //当前字符数，记录字符串中某一具体字符的下标；
				  i = 0;   //循环变量，一个字符由6个字节显示，故需要六次写入字节；
	{
		x--;
		y--;
		x *= 2;
		y *= 8;
	}//行列到点的转换；
    while (Data[count]!='\0')
    {
		if(y>127)
		{
			y=0;
			x++;
			x++;  //每行需要两个页；
		}    
        ascii_num = Data[count]-32;  //下标；     
        OLED_SetPosition(y,x);  //第一页；  
        for(i=0;i<8;i++)
		{     
            OLED_WriteByte(FONT_8x16[ascii_num*16+i]);
		}
        OLED_SetPosition(y,x+1); //第二页；   
        for(i=0;i<8;i++) 
		{    
            OLED_WriteByte(FONT_8x16[ascii_num*16+i+8]); 
		} 
        y+=8; //一个字符占用8个点；
        count++; //下个字符；
    }	
}

#endif


#if FONT_6x8_MODE //若定义了该宏，则表示6*8 ASCII码标准字体模式开启；
//6*8 字体模式显示函数；
/*
	 x: 行，y: 列 
	 对于6*8模式(一个字符占6个字节)而言 128*64的oled最多可以显示170个字符,每个字符
仅需要在一页中显示，故可以显示8行(页)，170/8=21，即每行可以显示21个字符，而且需要说
明的是，每行显示21个字节后还有两个点，即每行128个点只用到了126个点；
	8行21列
*/
void OLED_Show_FONT_6x8(unsigned char x , unsigned char y , unsigned char Data[])
{
	unsigned char ascii_num = 0, //字符转换成ASCII码后在字库中码表里的具体位置；
				  count = 0, //当前字符数，记录字符串中某一具体字符的下标；
				  i = 0;   //循环变量，一个字符由6个字节显示，故需要六次写入字节；
	{
		x--;
		y--; //由于输入是以我们较为符合的方式，故这里需
			//要转换，如：x为1时实际表示第一行，而这里必须是第0行
		y *= 6;
	}//行列到点的转换；
	while(Data[count] != '\0') //字符串还没结束；
	{
		if(y>=126)  //如果一行用完，跳到下一行的起始位；
		{
			y = 0;  //起始位；
			x++;	//下一页(行)；
		}
		ascii_num = Data[count] - 32; //字符转换成字库对应点阵的起始坐标；
		OLED_SetPosition(y,x); //设置显示点；
		for(i=0;i<6;i++) //写入一个字符；
		{
			OLED_WriteByte(FONT_6x8[ascii_num][i]);	
		}
		y += 6; //一列6个字节；
		count++;		
	}	
}
#endif

#if PICTURE_MODE  //若定义了该宏，则表示图片显示模式开启；
	//128*64 图片模式显示函数；
	/*
		( x0 , y0 ) 初始坐标，x0: 起始行(1~8)   y0: 起始列(0~128);
		( x1 , y1 ) 终点坐标，x1: 终点行(1~8)   y1: 终点列(0~128);
		自行根据规范使用，不按规范使用时没有提供错误处理功能；
	*/
	void OLED_Show_PICTURE_128x64(unsigned char x0 , unsigned char y0,
								  unsigned char x1 , unsigned char y1,unsigned int Picture_Index)
	{
		unsigned char x,y; //记录图片当前点的坐标；
		unsigned int count = Picture_Index; //记录第几幅图片的起始下标,(图片由很多字节组成)；
		{
			x0--;
			y0--;
			x1--;
			y1--;
		}
		for(x=x0;x<=x1;x++) //行；
		{
			/* 行包含列； */
			OLED_SetPosition(y0,x); //x行的起始位置；
			for(y=y0;y<=y1;y++) //列；
			{
				OLED_WriteByte(PICTURE_128x64[count++]);	
			}
		}	
	}
////////////////////////////////////////////////////////////////////////////////////////////
#endif
unsigned char oled[8]={0};
void OledShowData_float(float axis,unsigned char HANG,unsigned char LIE)
{	
  unsigned int temp_data=0;
  if(axis<0)
	{
	  axis=(-axis);
          oled[0]='-';
	}
        else
	{
	   oled[0]=' ';
	}
		 temp_data=(unsigned int)(axis*100);
		 oled[1]=temp_data/10000+0x30;
		 temp_data=temp_data%10000; 
		 oled[2]=temp_data/1000+0x30;
		 temp_data=temp_data%1000;
     oled[3]=temp_data/100+0x30;
		 oled[4]='.';	
     temp_data=temp_data%100;	
     oled[5]=temp_data/10+0x30;
     temp_data=temp_data%10;		
		 oled[6]=temp_data+0x30;
	   OLED_Show_FONT_6x8(HANG,LIE,oled);
}	 


