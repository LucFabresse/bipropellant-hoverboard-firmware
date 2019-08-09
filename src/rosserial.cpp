#ifdef __cplusplus
extern "C" {
#endif

#include "config.h"
#include "comms.h"
#include "bldc.h"
#include "hallinterrupts.h"

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

#define DELAY_MS_2HZ 500 // ms => 0.5s => 2Hz
#define DELAY_MS_10HZ 100 // ms
#define DELAY_MS_100HZ 10 // ms 
#define DELAY_MS_200HZ 5 // ms

long last2Hz_ms = timeStats.now_ms;	// ms
// long last10Hz_ms = timeStats.time_in_ms;	// ms
// long last100Hz_ms = timeStats.time_in_ms;	// ms
// long last200Hz_ms = timeStats.time_in_ms;	// ms

unsigned int rosserialLoopCount = 0;
char debugMsg [128];

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatterPublisher("chatter", &str_msg);

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  Buzzer Topic
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//TODO:  give the possibility to directrly configure buzzerFreq adn buzzerDelay from ROS
extern uint32_t buzzerFreq;
#define BUZZER_DELAY_MS 300

void buzzerBip() {
	buzzerFreq = 5;
	HAL_Delay(BUZZER_DELAY_MS);
	buzzerFreq = 0;
}

void buzzerBipCount(uint16_t bipCount) {
	for (int i = 0; i < bipCount; i++) {
		buzzerBip();
		HAL_Delay(BUZZER_DELAY_MS);
	}
}
 
void buzzerTopicCallback(const std_msgs::UInt16& bipCount){
	buzzerBipCount(bipCount.data);
}

ros::Subscriber<std_msgs::UInt16>subBuzzerTopic("buzzer_count", &buzzerTopicCallback );

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Battery Topic
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

std_msgs::Float32 batteryVoltageMsg;
ros::Publisher batteryVoltagePublisher("battery_voltage", &batteryVoltageMsg);

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Motors Topics Subscribers
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// void MotorLeftCb(const std_msgs::Int16& rpm) {
// 	motor_left_rpm(rpm.data);
// 	// sprintf(log_msg, "motor left data received %d", rpm.data);
// 	// nh.loginfo(log_msg);
// }
//
// void MotorRightCb (const std_msgs::Int16& rpm) {
// 	// nh.loginfo("motor right data received");
// 	motor_right_rpm(rpm.data);
// 	// sprintf(log_msg, "motor right data received %d", rpm.data);
// 	// nh.loginfo(log_msg);
// }

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Hall Topic Publishers
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 
// rosserial_init
// 
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	

void rosserial_init() {
	nh.initNode();

	nh.loginfo("[STARTING] rosserial topics creation\n");
	consoleLog("[STARTING] rosserial topics creation\n\r");	
	
	// str_msg.data = "Hello from Hoverboard\n\r";
	// nh.advertise(chatterPublisher);
	
	// subscribe /buzzer
	nh.subscribe(subBuzzerTopic);

	// Publish Topic /battery_voltage
	// TODO: replace by const sensor_msgs::BatteryState& 
	nh.advertise(batteryVoltagePublisher);

	// ros::Subscriber<std_msgs::Int16> subMotorLeftTopic("motor_left_rpm", &MotorLeftCb );
	// nh.subscribe(subMotorLeftTopic);
	//
	// ros::Subscriber<std_msgs::Int16> subMotorRightTopic("motor_right_rpm", &MotorRightCb );
	// nh.subscribe(subMotorRightTopic);
	// 
	// std_msgs::Int32 hallLeftMsg;
	// ros::Publisher hallLeftPublisher("motor_left_hall", &hallLeftMsg);
	// nh.advertise(hallLeftPublisher);
	//
	// std_msgs::Int32 hallRightMsg;
	// ros::Publisher hallRightPublisher("motor_right_hall", &hallRightMsg);
	// nh.advertise(hallRightPublisher);
	//
	// std_msgs::Int32 ticksRightMsg;
	// ros::Publisher ticksRightPublisher("motor_right_ticks", &ticksRightMsg);
	// nh.advertise(ticksRightPublisher);
	//
	// std_msgs::Int32 ticksLeftMsg;
	// ros::Publisher ticksLeftPublisher("motor_left_ticks", &ticksLeftMsg);
	// nh.advertise(ticksLeftPublisher);
	
	nh.loginfo("[OK] rosserial topics created\n");		
	consoleLog("[OK] rosserial topics created\n\r");	
}


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 
// rosserial_loop
// 
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	

void rosserial_loop() {
		
	// hallLeftMsg.data = motor_left_hall();
	// hallLeftPublisher.publish(&hallLeftMsg);
	//
	// hallRightMsg.data = motor_right_hall();
	// hallRightPublisher.publish(&hallRightMsg);
	//
	// ticksRightMsg.data = motor_right_ticks();
	// ticksRightPublisher.publish(&ticksRightMsg);
	//
	// ticksLeftMsg.data = motor_left_ticks();
	// ticksLeftPublisher.publish(&ticksLeftMsg);
	
	// consoleLog("looping\r\n");
	
	if(timeStats.now_ms - last2Hz_ms > DELAY_MS_2HZ ) {
		last2Hz_ms = timeStats.now_ms;
		batteryVoltageMsg.data = electrical_measurements.batteryVoltage;
		batteryVoltagePublisher.publish(&batteryVoltageMsg);
		
		sprintf(debugMsg,"[%u] rosserial heartbeat\n\r",rosserialLoopCount);
		consoleLog(debugMsg);
	}
	
	// USART2_IT_send((unsigned char*)"USART2_IT_send\n\r",(int)16);
	// USART3_IT_send((unsigned char*)"USART3_IT_send\n\r",(int)16);
	
	nh.spinOnce();
	rosserialLoopCount++;
}
