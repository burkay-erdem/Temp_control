#include  "Max6675.h"
#define High 1
#define Low 0

float err=0 , integral = 0, derivative = 1 ,last_error=0;

extern SPI_HandleTypeDef hspi2;
void max6675_init(max6675_pin *gpio){
//	HAL_GPIO_WritePin(gpio->CLK_PORT,gpio->CLK_PIN,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(gpio->Cs_PORT,gpio->Cs_PIN,GPIO_PIN_SET);

}


float Max6675_read(max6675_pin *gpio){

HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
	
	HAL_Delay(1);
	uint16_t value =0 ;
	
	value=Spi_read(gpio);
	value <<= 8;
	value |= Spi_read(gpio);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
	if(value & 0x4)
return 1;
	value >>= 3 ;
	return value * 0.25;
}

uint8_t Spi_read(max6675_pin *gpio){
	
	int i ;
	uint8_t d =0 ;
	for(i =7 ; i>=0;i--){
	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
		HAL_Delay(1);
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7))
			d|=(1<<i);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
		HAL_Delay(1);
	}
}


float Max6675_Read(){
	uint8_t highbyte[1],lowbyte[1];
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
	
	HAL_Delay(10);
	uint16_t value =0 ;
	
	HAL_SPI_Receive(&hspi2,highbyte,1,100);
		HAL_Delay(1);
	value = highbyte[0];	
	HAL_SPI_Receive(&hspi2,lowbyte,1,100);	

	value <<= 8;
	value |= lowbyte[0];
	
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
	HAL_Delay(1);
	if(value & 0x4)
	return 1;
	else{
	value >>= 3 ;
	return 0.25 *value;
	}
}

float PID_stm32 (float ki , float kp , float kd ,float set_value ,float now){

int now_value = now;
	HAL_Delay(10);
	
	err = set_value-now_value;
	integral += err;
	derivative = err-last_error;
	last_error = err;
	
return ki*integral+kp*err+kd*derivative; 
}

void read_Pid (max6675_pin gpio){

HAL_GPIO_TogglePin(gpio.gpio,gpio.pin);
	HAL_Delay(500);

}
