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

#define DELAY_MS_1HZ 1000 // ms
#define DELAY_MS_2HZ 500 // ms => 0.5s => 2Hz
#define DELAY_MS_10HZ 100 // ms
#define DELAY_MS_100HZ 10 // ms 
#define DELAY_MS_200HZ 5 // ms

long last1Hz_ms = timeStats.now_ms;	// ms
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

extern volatile uint32_t input_timeout_counter; // global variable for input_timeout

// if not called regularly, motors stop
void signalHeartBeat() {
	input_timeout_counter = 0;
}

// #define LOG_PWM

// #define ROSPWMLIMIT 1000
#define ROSPWMLIMIT 200

// only for propellent protocols (Machine, ASCII, ...)
// control_type = CONTROL_TYPE_PWM;		// CONTROL_TYPE_NONE CONTROL_TYPE_PWM CONTROL_TYPE_SPEED CONTROL_TYPE_POSITION CONTROL_TYPE_MAX
extern int control_type;

extern PROTOCOL_PWM_DATA PWMData;

void motorLeftTopicCallback(const std_msgs::Int16& pwmLeft) {
	control_type = CONTROL_TYPE_PWM;
	PWMData.pwm[0] = CLAMP(pwmLeft.data, -ROSPWMLIMIT, ROSPWMLIMIT);
	signalHeartBeat();
	// sprintf(debugMsg,"/lmotor received %d -> set %d\n",pwmLeft.data,PWMData.pwm[0]);
	// nh.loginfo(debugMsg);
}
ros::Subscriber<std_msgs::Int16> subMotorLeftTopic("lmotor", &motorLeftTopicCallback);

void motorRightTopicCallback(const std_msgs::Int16& pwmRight) {
	control_type = CONTROL_TYPE_PWM;
	PWMData.pwm[1] = CLAMP(pwmRight.data, -ROSPWMLIMIT, ROSPWMLIMIT);
	signalHeartBeat();
	// sprintf(debugMsg,"/rmotor received %d -> set %d\n",pwmRight.data,PWMData.pwm[1]);
	// nh.loginfo(debugMsg);
}
ros::Subscriber<std_msgs::Int16> subMotorRightTopic("rmotor", &motorRightTopicCallback);

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
	
	// init HeartBeat
   input_timeout_counter = 0;
	
	// init motors variables
	PWMData.pwm[0] = 0;
   PWMData.pwm[1] = 0;
	
	nh.loginfo("[STARTING] rosserial topics creation\n");
	
	// subscribe /buzzer
	nh.subscribe(subBuzzerTopic);

	// Publish Topic /battery_voltage
	// TODO: replace by const sensor_msgs::BatteryState& 
	nh.advertise(batteryVoltagePublisher);

	// how we control motors
	nh.subscribe(subMotorLeftTopic);
	nh.subscribe(subMotorRightTopic);

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
	
	if(timeStats.now_ms - last2Hz_ms > DELAY_MS_2HZ ) {
		last2Hz_ms = timeStats.now_ms;
						
		batteryVoltageMsg.data = electrical_measurements.batteryVoltage;
		batteryVoltagePublisher.publish(&batteryVoltageMsg);
	}
	
	// How to send things on USARTs
	// consoleLog(debugMsg);
	// USART2_IT_send((unsigned char*)"USART2_IT_send\n\r",(int)16);
	// USART3_IT_send((unsigned char*)"USART3_IT_send\n\r",(int)16);
	
	nh.spinOnce();
	rosserialLoopCount++;
}
