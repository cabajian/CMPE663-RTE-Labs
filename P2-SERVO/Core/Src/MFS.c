/*
 * MFS.c
 *
 *  Created on: Aug 31, 2021
 *      Author: rickweil
 */
#include "stm32l476xx.h"
#include "MFS.h"

//////////////////////////////////////////
// private variables

static GPIO_OUT_t leds[] =
{
	{ .port=GPIOA, .pin=5, .otype=0, .ospeed=0, .init_value=1},	// place holder
	{ .port=GPIOA, .pin=5, .otype=0, .ospeed=0, .init_value=1},	// LED 1 d13 PA5
	{ .port=GPIOA, .pin=6, .otype=0, .ospeed=0, .init_value=1},	// LED 2 d12 PA6
	{ .port=GPIOA, .pin=7, .otype=0, .ospeed=0, .init_value=1},	// LED 3 d11 PA7
	{ .port=GPIOB, .pin=6, .otype=0, .ospeed=0, .init_value=1},	// LED 4 d10 PB6
};

static GPIO_IN_t buttons[] =
{
	{ .port=GPIOA, .pin=1, .pupd=0},	// place holder
	{ .port=GPIOA, .pin=1, .pupd=0},	// Button 1 a2 PA1
	{ .port=GPIOA, .pin=4, .pupd=0},	// Button 2 a2 PA4
	{ .port=GPIOB, .pin=0, .pupd=0},	// Button 3 a3 PB0
};

#define SEG7 1
#if SEG7
static GPIO_OUT_t seg7[] = {
	{ .port=GPIOA, .pin=9, .otype=0, .ospeed=0, .init_value=0},	// data	 d8  PA9
	{ .port=GPIOA, .pin=8, .otype=0, .ospeed=0, .init_value=0},	// shift d7  PA8
	{ .port=GPIOB, .pin=5, .otype=0, .ospeed=0, .init_value=0},	// latch d12 PB5
};

/// write 8 bits of `value` to shift register
static void shiftOut(uint8_t value) {
	for(int ii=0x80; ii; ii>>=1) {

		GPIO_TypeDef *clock_port = seg7[1].port;
		uint32_t clock_bit = 1 << (seg7[1].pin);

		GPIO_TypeDef *data_port = seg7[0].port;
		uint32_t data_bit = 1 << (seg7[0].pin);

		clock_port->ODR &= ~clock_bit;		// clear clock pin
		if(ii && value)						// if this bit in `value` is set
			data_port->ODR |= data_bit;		//   set it in shift register
		else
			data_port->ODR  &= ~data_bit;	//   else clear it
		clock_port->ODR |= clock_bit;		// set clock pin
	}
}

static const uint8_t SEGMENT_SELECT[] = {0xF1,0xF2,0xF4,0xF8};  // low selects the letter (0-3)
/// write `segments` to drive letter `n`
static void set_segment(uint8_t n, uint8_t segments)
{
	GPIO_TypeDef *latch_port = seg7[2].port;
	uint32_t latch_bit = 1 << (seg7[2].pin);

	latch_port->ODR &= ~latch_bit;		// clear latch pin low
	shiftOut(segments);					// write 8 bits for this digit
	shiftOut(SEGMENT_SELECT[n%4]);		// and select which digit
	latch_port->ODR |= latch_bit;		// set latch pin
}
#endif


//////////////////////////////////////////
// private functions

static void gpio_config_input(GPIO_IN_t *gpio)
{
	GPIO_TypeDef *port = gpio->port;
	uint32_t pin = gpio->pin;

	// First, configure as input
    port->MODER &= ~(0x3 << (pin*2)) ;      // clear the two MODE bits for this pin
    port->MODER |=  0 << (pin*2)  ;        	// 0 => input

	port->PUPDR &= ~(0x3 << (pin*2)) ;      // clear the two PUPDR bits for this pin
	port->PUPDR |=  gpio->pupd << (pin*2)  ;
}

static void gpio_config_output(GPIO_OUT_t *gpio)
{
	GPIO_TypeDef *port = gpio->port;
	uint32_t pin = gpio->pin;

	// First, configure as an output
    port->MODER &= ~(0x3 << (pin*2)) ;      // clear the two MODE bits for this pin
    port->MODER |=  1 << (pin*2)  ;        	// 1 => output

	// ...and then the selected drive
	port->OTYPER &= ~(0x1 << pin) ;
	port->OTYPER |= (gpio->otype << pin) ;

	// ...with selected speed
	port->OSPEEDR &= ~(0x3 << (pin*2)) ; 	// clear the two OSPEED bits for this pin
	port->OSPEEDR |= gpio->ospeed << (pin*2) ;

	// ...set initial value
	port->ODR &= ~(0x1 << pin);
	port->ODR |= (gpio->init_value << pin);
}

//////////////////////////////////////////
// public functions

void MFS_init(void)
{
	// configure the LEDs as GPIO outputs
	for(int ii=1; ii<=4; ii++) {
		gpio_config_output(&leds[ii]);
	}

	// configure the buttons as GPIO inputs
	for(int ii=1; ii<=3; ii++) {
		gpio_config_input(&buttons[ii]);
	}
#if SEG7
	// configure data, clock and latch of the 7-segment display as GPIO outputs
	for(int ii=0; ii<3; ii++) {
		gpio_config_output(&seg7[ii]);
	}
	// then blank all 4 digits
	for(int ii=0; ii<4; ii++) {
		set_segment(ii, 0xff);
	}
#endif
}

// Turns LED `num` to 'on' if non-zero, or off if zero
void MFS_set_led( uint8_t num, uint32_t on )
{
	while(num > 4);	// hang if invalid input

	GPIO_TypeDef *port = leds[num].port;
	uint32_t bit = 1 << leds[num].pin;

	if ( on )
		port->ODR &= ~bit ;	// active low
	else
		port->ODR |= bit ;
}

// Toggles LED `num`
void MFS_toggle_led( uint8_t num )
{
	while(num > 4);	// hang if invalid input

	GPIO_TypeDef *port = leds[num].port;
	uint32_t bit = 1 << leds[num].pin;

	if ( port->IDR & bit )
		port->ODR &= ~bit ;
	else
		port->ODR |= bit ;
}

// Returns 1 if button `num` if pressed, 0 otherwise
uint8_t MFS_button_pressed( uint8_t num )
{
	while(num > 3);	// hang if invalid input

	GPIO_TypeDef *port = buttons[num].port;
	uint32_t bit = 1 << buttons[num].pin;

	return (port->IDR & bit) == 0;  // invert active low
}
