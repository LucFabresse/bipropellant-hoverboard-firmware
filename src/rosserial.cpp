#ifdef __cplusplus
extern "C" {
#endif

#include "config.h"
#include "comms.h"
		
#ifdef __cplusplus
}
#endif

#include "rosserial_stm32/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// rosserial globals
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatterPublisher("chatter", &str_msg);
int logCount = 0;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// rosserial_init
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	
void rosserial_init() {
	nh.initNode();
	str_msg.data = "Hello from Hoverboard\n\r";
	nh.advertise(chatterPublisher);
	nh.loginfo("[OK] Hoverboard rosserial started\n");
		
	consoleLog("[OK] Hoverboard rosserial [USART3]\n\r");	
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// rosserial_loop
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	
void rosserial_loop() {
	chatterPublisher.publish(&str_msg);
	nh.spinOnce();

	if(logCount++ % 300 == 0) {
		USART2_IT_send((unsigned char*)"USART2_IT_send\n\r",(int)16);
		USART3_IT_send((unsigned char*)"USART3_IT_send\n\r",(int)16);					 	
		consoleLog("ConsoleLog\n\r");	
	}
}
