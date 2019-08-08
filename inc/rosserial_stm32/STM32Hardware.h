/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, Kenta Yonekura (a.k.a. yoneken)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROS_STM32_HARDWARE_H_
#define ROS_STM32_HARDWARE_H_

// Black magic cast to remove volatile. DO NOT TOUCH!!!
// #define tbuf  ((uint8_t *)&uart.TX_buffer[0])

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"
// #include "stm32f1xx_hal_tim.h"

extern "C" {
	#include "comms.h"
}

extern UART_HandleTypeDef huart2;
extern volatile SERIAL_USART_BUFFER usart2_it_RXbuffer;


class STM32Hardware {
  protected:
    UART_HandleTypeDef *huart;
 
  public:
    STM32Hardware():
      huart(&huart2) {
    }

    STM32Hardware( UART_HandleTypeDef *huart_):
     huart(huart_) {
    }
  
  	 // Ros::NodeHandle callback
    void init(){
      // nothing there
    }

	 int read(){
	 	int rx_value = -1;

		if ( serial_usart_buffer_count(&usart2_it_RXbuffer) > 0 ) {
			rx_value = serial_usart_buffer_pop(&usart2_it_RXbuffer);
	 	}

	 	return rx_value;
	 }

	 void flush(void){
		 // nothing there
	 }
	 
	 void write(uint8_t* data, int length){
		 USART2_IT_send((unsigned char *)data,length);
	 }

    unsigned long time(){ return HAL_GetTick(); }

  protected:
};

#endif

