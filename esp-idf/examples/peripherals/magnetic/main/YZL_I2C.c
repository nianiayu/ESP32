#include "time.h"
#include "sys/time.h"
#include "YZL_I2C.h"


/*#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "time.h"
#include "sys/time.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/gpio_sig_map.h"
#include "soc/io_mux_reg.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/periph_ctrl.h"

#include "I2C.h"*/
/*****************************************************************************/
void esp32_delay_1us(void)
{
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");//10
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");//20
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");//30
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");//40
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");//50

	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");//60
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");//70
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");//80
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");//90
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");//100
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");//110
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");//120
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");//130
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");//140
}
/*****************************************************************************/
void delay_us(unsigned  int num)
{
		unsigned  int i;
		for(i=0; i<num; i++)
		{
				esp32_delay_1us();
		}
}
/*****************************************************************************/
/*****************************************************************************/
/*������Ч�ԣ����˿�ʼ��ֹͣ״̬�� �����ݴ�������У��� SCL Ϊ�ߵ�ƽʱ�����뱣֤ SDA �ϵ������ȶ���
Ҳ����˵�� SDA �ϵĵ�ƽ�任ֻ�ܷ����� SCL Ϊ�͵�ƽ��ʱ�� SDA ���ź��� SCL Ϊ�ߵ�ƽʱ���ɼ���*/
/*****************************************************************************/
//������I2C�ײ�����
/*****************************************************************************/
//����I2C��ʼ�ź�:SCLΪ�ߵ�ƽʱ,SDA����һ���½���
void IRAM_ATTR I2C_start(void)
{
		I2C_SDA_OUT() ;   //sda�����
		I2C_SCL_OUT() ;  	  
		I2C_SCL_OUT_H ;
		I2C_SDA_OUT_H;	
		delay_us(8);            //delay_us(5)---���� 4us
		I2C_SDA_OUT_L;
		delay_us(8);
		I2C_SCL_OUT_L;//ǯסI2C���ߣ�׼�����ͻ�������� 
}
/*****************************************************************************/
//����I2Cֹͣ�ź�:SCLΪ�ߵ�ƽʱ,SDA����һ��������
void IRAM_ATTR I2C_stop(void)
{
		I2C_SDA_OUT();   //sda�����
		I2C_SCL_OUT();
		I2C_SDA_OUT_L;//START:when CLK is high,DATA change form high to low // 	delay_us(4);
		I2C_SCL_OUT_H;
		delay_us(8);
		I2C_SDA_OUT_H;	  	  						   	
}
/*****************************************************************************/
//�ȴ�Ӧ���źŵ���
//����ֵ��1--����Ӧ��ʧ��;0--����Ӧ��ɹ�
uint8_t IRAM_ATTR I2C_wait_ACK(void) //ACK:SDA LOW
{
		unsigned  int ack=0;
		I2C_SDA_IN();   //SDA����Ϊ����     
		I2C_SCL_OUT_H;
		delay_us(8); 
		ack = I2C_READ_SDA();
		I2C_SCL_OUT_L;//ʱ�����0 
		delay_us(8);	
		return ack;  
} 
/*****************************************************************************/
//����ACKӦ�� 0 ack  1 noack
void IRAM_ATTR I2C_ACK(uint8_t ack )   //NO ACK:SDA HIGH
{
		I2C_SDA_OUT();
		if(ack)
		{
				I2C_SDA_OUT_H;
		}
		else
		{
				I2C_SDA_OUT_L;
		}
		I2C_SCL_OUT_H;
		delay_us(8);
		I2C_SCL_OUT_L;
}
/*****************************************************************************/
void IRAM_ATTR I2C_send_byte(uint8_t txd)
{                        
		uint8_t t;   
		I2C_SDA_OUT() ;  
		for(t=0;t<8;t++)
		{     
				gpio_set_level(I2C_SDA_GPIO,(txd&0x80)>>7);		
				txd<<=1; 	  
				I2C_SCL_OUT_H;
				delay_us(20);   
				I2C_SCL_OUT_L;
				delay_us(20); 
		}	 
		I2C_wait_ACK();
} 
/*****************************************************************************/
//I2C��һ���ֽ�
uint8_t IRAM_ATTR I2C_read_byte(void)
{
		uint8_t i,receive=0;
		I2C_SDA_OUT_H;
		I2C_SDA_IN() ;             //SDA����Ϊ����
		for( i=0 ; i<8; i++ )
		{
				receive <<=  1;
				I2C_SCL_OUT_H ;
				delay_us(20);
				if(I2C_READ_SDA())
						receive++;				
				I2C_SCL_OUT_L ; 
				delay_us(20);	
		}					 
		return receive;
}
/*****************************************************************************/
void IRAM_ATTR I2C_Single_Write(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data)
{
		I2C_start();                              //��ʼ�ź�
		I2C_send_byte(SlaveAddress);    //�����豸��ַ+д�ź�
		I2C_send_byte(REG_Address);    //�ڲ��Ĵ�����ַ
		I2C_send_byte(REG_data);        //�ڲ��Ĵ�������
		I2C_stop();                              //����ֹͣ�ź�
}
/*****************************************************************************/
void IRAM_ATTR I2C_continuous_write(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t *pw,uint8_t quantity)		
{
	uint8_t i;	
	for(i=0;i<=quantity;)
	{
		I2C_start();
		I2C_send_byte(SlaveAddress);         
		I2C_send_byte(REG_Address);        
		for(;i<=quantity;i++)                       
		{
			I2C_send_byte(*pw++); 	  //�ڲ��Ĵ�������		
			REG_Address++;
			delay_us(50); 
		}
		I2C_stop();        
		delay_us(20000);  
	}
}
/*****************************************************************************/
uint8_t IRAM_ATTR I2C_Single_Read(uint8_t SlaveAddress,uint8_t REG_Address)  //��һ���ֽڵ�����
{
		uint8_t REG_data;
		I2C_start();                                //��ʼ�ź�
		I2C_send_byte(SlaveAddress);      //�����豸��ַ+д�ź�
		I2C_send_byte(REG_Address);      //���ʹ洢��Ԫ��ַ����0��ʼ	
		I2C_start();                                //��ʼ�ź�
		I2C_send_byte(SlaveAddress+1);  //�����豸��ַ+���ź�
		REG_data=I2C_read_byte();         //�����Ĵ�������
		I2C_ACK(1);                               //����Ӧ���ź�
		I2C_stop();                                //ֹͣ�ź�
		return REG_data;
}
/*****************************************************************************/
void IRAM_ATTR I2C_Read_Bytes(uint8_t SlaveAddress,uint8_t REG_Address, uint8_t *str,uint8_t length) //������оƬ�����ݡ�
{
		uint8_t i = 0;
		I2C_start();                                 //��ʼ�ź�
		I2C_send_byte(SlaveAddress);       //�����豸��ַ+д�ź�ss
		I2C_send_byte(REG_Address);
		I2C_start();  
		I2C_send_byte(SlaveAddress+1);   //�����豸��ַ+���ź�	
		for(i = 0;i < length-1 ;i++)
		{
				*str=I2C_read_byte();          //�����Ĵ�������
				I2C_ACK(0);
				str ++ ;
				delay_us(20);
		}
		*str=I2C_read_byte();     //�����Ĵ������� 
		I2C_ACK(1);	
		I2C_stop();                    //ֹͣ�ź�
}
/*****************************************************************************/
void IRAM_ATTR I2C_16Addr_ReadBytes(uint8_t SlaveAddress,uint16_t REG_Address,uint8_t *str,uint8_t length) //������оƬ�����ݡ�
{
		unsigned char AddrH, AddrL;
		uint8_t i = 0;
		AddrH = REG_Address >> 8;
		AddrL = REG_Address & 0x00ff;
		I2C_start();                                 //��ʼ�ź�
		I2C_send_byte(SlaveAddress);       //�����豸��ַ+д�ź�
		I2C_send_byte(AddrH);
		I2C_send_byte(AddrL);
		I2C_start();  
		I2C_send_byte(SlaveAddress+1);   //�����豸��ַ+���ź�	
		for(i = 0;i < length-1 ;i++)
		{
				*str=I2C_read_byte();           //�����Ĵ�������
				I2C_ACK(0);
				str ++ ;
				delay_us(20);
		}
		*str=I2C_read_byte();     //�����Ĵ������� 
		I2C_ACK(1);	
		I2C_stop();                    //ֹͣ�ź�
}
/*****************************************************************************/
