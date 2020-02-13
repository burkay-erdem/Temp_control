#include "stm32f4xx.h"


typedef struct {

GPIO_TypeDef* gpio;
 uint16_t pin;
	
	 uint16_t pine;
}max6675_pin;
float Max6675_read(max6675_pin *gpio_pins);
void Max6675_init(max6675_pin *gpio_pins);
uint8_t Spi_read(max6675_pin *gpio_pins);
float Max6675_Read();

float PID_stm32 (float ki , float kp , float kd ,float set_value,float now);

void read_Pid (max6675_pin gpio);