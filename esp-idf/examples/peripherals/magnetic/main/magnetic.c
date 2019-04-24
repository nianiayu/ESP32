/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "YZL_I2C.h"
#include "YZL_I2C.c"

#define QMC_X953 0

/* Can run 'make menuconfig' to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

int one_shot = 1;
#define QMC5883_ADDR 0X1A

#if QMC_X953
     #define ST480_I2C_ADDRESS 0x58
#else
     #define ST480_I2C_ADDRESS 0x18
#endif

#define SINGLE_MEASUREMENT_MODE_CMD 0x3E
#define READ_MEASUREMENT_CMD 0x4E

int32_t offset_x, offset_y, offset_z;

void i2c_test(void *pvParameter)
{

	uint8_t data[9] = { 0 };
	int cnt = 0;
	
	
	while(1)
	{
		printf("now, yzl do i2c test address=0x1A\n");

		 I2C_Single_Read( ST480_I2C_ADDRESS, SINGLE_MEASUREMENT_MODE_CMD); 
		
        delay_us(1000*40);
		

	    I2C_Read_Bytes(ST480_I2C_ADDRESS, READ_MEASUREMENT_CMD, data , 9);

		
        delay_us(1000*40);

		for(cnt=0; cnt<9; cnt++)
		{
			printf("msensor data: data[%d]=%02x\n", cnt, data[cnt]);
		}

	}


}






void qmc5883_task(void *pvParameter)
{
	 uint8_t r[6] = {0}, mode = 0;
	 int mag[3]={0};
	 char temp;
	 
	I2C_Single_Write(QMC5883_ADDR, 0X0B, 0X01);
	I2C_Single_Write(QMC5883_ADDR, 0X09, 0X1D);
	delay_us(30*1000);


	while(1) 
	{

		I2C_Read_Bytes(QMC5883_ADDR,  0x00,  r, 6);
		mag[0] = (int)(int16_t)( (r[1]<<8) | (r[0]));
		mag[1] = (int)(int16_t)( (r[3]<<8) | (r[2]));
		mag[2] = (int)(int16_t)( (r[5]<<8) | (r[4]));

		printf(" %d  %d  %d\n", mag[0],  mag[1], mag[2] );

		//temp = I2C_Single_Read(QMC5883_ADDR, 0x09);
		//printf("FIRST reg[9]=%X\n", temp);


		temp = I2C_Single_Read(QMC5883_ADDR, 0x09);
		printf("END reg[9]=%X\n", temp);


	//	delay_us(300*1000);
		vTaskDelay(20 / portTICK_RATE_MS);

	 }

	   
}

void getData(int16_t *x, int16_t *y, int16_t *z)
{
 
        uint8_t  r[7] = {0};
	int16_t mag[3]={0};

	I2C_Read_Bytes(ST480_I2C_ADDRESS,  0x4E,  r, 7);
	mag[0] = r[1]<<8 | r[2];
	mag[1] = r[3]<<8 | r[4];
	mag[2] = r[5]<<8 | r[6];

        *x = mag[0]*15/100;
        *y = mag[1]*15/100;
        *z = mag[2]*25/100;
//	printf("nick read ret=%X\n", r[0]);
	printf("mag data= %d, %d, %d\n", *x, *y, *z);

	I2C_Read_Bytes(ST480_I2C_ADDRESS,  0x3E,  r, 1);
//	printf("single mode=%X\n", r[0]);
	
	vTaskDelay(40 / portTICK_RATE_MS);


}

uint32_t count = 0;
void calibrate(uint32_t timeout, int32_t *offsetx, int32_t *offsety, int32_t*offsetz)
{
  int32_t value_x_min = 0;
  int32_t value_x_max = 0;
  int32_t value_y_min = 0;
  int32_t value_y_max = 0;
  int32_t value_z_min = 0;
  int32_t value_z_max = 0;
  uint32_t timeStart = 0;
  int16_t x,y,z;

  getData(&x, &y, &z);

  value_x_min = x;
  value_x_max = x;
  value_y_min = y;
  value_y_max = y;
  value_z_min = z;
  value_z_max = z;
  //delay(100);

 // timeStart = millis();
  
  //while((millis() - timeStart) < timeout)
  while(count<1000)
   {
    count++;

    printf("count = %d\n", count);
    getData(&x, &y, &z);
    
    /* Update x-Axis max/min value */
    if(value_x_min > x)
    {
      value_x_min = x;
      // Serial.print("Update value_x_min: ");
      // Serial.println(value_x_min);

    } 
    else if(value_x_max < x)
    {
      value_x_max = x;
      // Serial.print("update value_x_max: ");
      // Serial.println(value_x_max);
    }

    /* Update y-Axis max/min value */
    if(value_y_min > y)
    {
      value_y_min = y;
      // Serial.print("Update value_y_min: ");
      // Serial.println(value_y_min);

    } 
    else if(value_y_max < y)
    {
      value_y_max = y;
      // Serial.print("update value_y_max: ");
      // Serial.println(value_y_max);
    }

    /* Update z-Axis max/min value */
    if(value_z_min > z)
    {
      value_z_min = z;
      // Serial.print("Update value_z_min: ");
      // Serial.println(value_z_min);

    } 
    else if(value_z_max < z)
    {
      value_z_max = z;
      // Serial.print("update value_z_max: ");
      // Serial.println(value_z_max);
    }
    
   // delay(100);

  }

  *offsetx = value_x_min + (value_x_max - value_x_min)/2;
  *offsety = value_y_min + (value_y_max - value_y_min)/2;
  *offsetz = value_z_min + (value_z_max - value_z_min)/2;
   
   printf("magnet offset=%d %d %d\n", *offsetx, *offsety, *offsetz);  

}
















double declination_shenzhen = -2.2;

void st480_task(void *pvParameter)
{

      int16_t x,y,z, r[7] = {0};
      
      int cnt = 0;
	int16_t mag[3]={0};
#if QMC_X953
while(1){	
	I2C_Single_Write(ST480_I2C_ADDRESS, 0X09, 0X1D);

	
	r[0] = I2C_Single_Read( ST480_I2C_ADDRESS, 0x09); 
        printf("reg[0x09] = %X\n", r[0]);

	
	vTaskDelay(1000 / portTICK_RATE_MS);
	}
#else
	r[0] = I2C_Single_Read( ST480_I2C_ADDRESS, 0x3E); 
	printf("init ret= %X\n", r[0]);

#endif
	 
	  
             printf("Start figure-8 calibration after 2 seconds.\n");

	     vTaskDelay(2000 / portTICK_RATE_MS);
            
	    printf("Start calibration...\n");
    	    calibrate(100000, &offset_x, &offset_y, &offset_z);


	while(1) {
		
		printf("offset: %d %d %d\n", offset_x, offset_y, offset_z);	
		
		getData(&x, &y, &z);
		x = x - offset_x;
   		y = y - offset_y;
   		z = z - offset_z;
		
		x = -x;

		printf("uT: %d %d %d\n", x, y, z);

    double pitch = 0, roll = 0;

    double Xheading = x * cos(pitch) + y * sin(roll) * sin(pitch) + z * cos(roll) * sin(pitch);
    double Yheading = y * cos(roll) - z * sin(pitch);
    

    double heading = 180 + 57.3*atan2(Yheading, Xheading) + declination_shenzhen;


    printf("heading = %f\n", heading); 











		vTaskDelay(1000/portTICK_RATE_MS);
    	}

	   
}

void app_main()
{
    //nvs_flash_init();
    
//    xTaskCreate(&qmc5883_task, "qmc5883_task", 2048, NULL, 5, NULL);
    xTaskCreate(&st480_task, "st480_task", 2048, NULL, 5, NULL);
}
