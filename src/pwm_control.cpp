
#include "ros/ros.h"
#include "pigpio.h"
#include "wiringPi.h"
#include "time.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <signal.h>

//#define PPR 55500 //pulses per revolution
#define GEAR_RATIO 111
#define NUM_REV 1
#define PULSE_PER_DEG 154
#define GPIO 18
#define DIR 24
#define ENCODERA 27
#define ENCODERB 22
#define PPM 	4
#define SOFTPWM 15
#define WALKASSIST 23
#define SPEEDOMETER 14

int vel_data = 0;

//Initial speed
int speed = 000000;


float currentAngle = 0.0;
double PPR = 55500.0;

int encoderA = 0;
int encoderB = 0;
int CW;

//Variables for the RC-controller part
unsigned int ppmChannelsNumber = 8;      // Number of channels packed in PPM
unsigned int ppmSyncLength     = 4000;   // Length of PPM sync pause
unsigned int currentChannel = 0;

unsigned int previousTick;
unsigned int deltaTime;

//Time variables for steering motor
unsigned int previousTick_angular;
unsigned int deltaTime_angular;

//Time variables for driving motor
unsigned int previousTick_linear;
unsigned int deltaTime_linear;

float frequencyDriveMotor;

float channels[8];

int stickVal = 0;
int stop = 0;

float angularSpeed = 0;
float linearSpeed = 0;




//Exit routine
void sigintHandler (int sig){
	
	
	//Turn off motors
	gpioPWM(SOFTPWM,0);
	gpioHardwarePWM(GPIO,50000,0);
	
	ROS_INFO("EXIT");
	//Exit gpio library
	gpioTerminate();
	//Exit ROS
	ros::shutdown();
}

void speedometerInterrupt(int gpio, int level, uint32_t tick){

	if(level == 1){
		
		deltaTime_linear = tick-previousTick_linear;
		frequencyDriveMotor = 1/(deltaTime/1000000);
		linearSpeed = 0.3566*frequencyDriveMotor-0.002715;
		previousTick_linear = tick;
	}

}

//Recieve RC-controller commands
void interruptPPM(int gpio, int level, uint32_t tick){
	
	if (level == 0) {	
		deltaTime = tick - previousTick;
		previousTick = tick;
		
		//Check when new message has arrived
		if (deltaTime >= ppmSyncLength) {
			currentChannel = 0;
		
				//Check left stick, values above 1510 will output as pointing to the right
				//values below 1490 will output as pointing to the left
				//deltaTime around 1500 is when the stick is in the middle, thus comparing to 					//1510 and 1490
				if(channels[3] > 1530 && stop == 0){									     ROS_INFO("[%lf]",currentAngle);
					//Right		
					stickVal = abs(channels[3]-1530);
					gpioWrite(DIR,1);
					if(currentAngle > 90){
						gpioHardwarePWM(GPIO,50000,0);
					}else{
						gpioHardwarePWM(GPIO,50000,stickVal*500);
					}
				}else if(channels[3] < 1470 && stop == 0){	
					//Left		
					ROS_INFO("[%lf]",currentAngle);	
					stickVal = abs(channels[3]-1470);
					gpioWrite(DIR,0);
					if(currentAngle < -90){
						gpioHardwarePWM(GPIO,50000,0);
					}else{
						gpioHardwarePWM(GPIO,50000,stickVal*500);
					}
					
				}
				
				//Check right stick, values above 1510 will output as pointing up
				//values below 1490 will output as pointing down		
				//deltaTime 1500 = stick is in middlepoint		
				if(channels[1] < 1490){			
					//Down		
					gpioPWM(SOFTPWM,0);
					
				}else if(channels[1] > 1510 && stop == 0){		
					//Up			
					gpioPWM(SOFTPWM,128);
					
				}
				
				//SWA lever on the controller, up is on down is off
				if(channels[4] < 1500){
					//Motors OFF
					gpioPWM(SOFTPWM,0);
					gpioHardwarePWM(GPIO,50000,0);
					stop = 1;
					
					
				}else{
					//Motors ON
					stop = 0;
				}
				
				//SWB lever on the controller, up is 0 middle is 1 and down is 2
				if(channels[5] < 1200){
					
				}else if (channels[5] > 1200 && channels[5] < 1700){
					
				}else{
					
				}
				
				//SWC lever on the controller, up is 0 middle is 1 and down is 2
				if(channels[6] < 1200){
					
				}else if (channels[6] > 1200 && channels[6] < 1700){
					
				}else{
					
				}		
				
				//SWD lever on the controller, up is 0 down is 1
				if(channels[7] < 1500){
					gpioWrite(WALKASSIST,1);
				}else{
					gpioWrite(WALKASSIST,0);
				}			
		}
		//If between to sync pulses we fill the channel array with values for each channel
		else{			
			if (currentChannel < ppmChannelsNumber){
				channels[currentChannel++] = deltaTime;
			}
		}
	}
}


//Recieve encoder data from channel A. Enters when the level changes on the GPIO pin
void encoderInterruptA(int gpio, int level, uint32_t tick){
	encoderA = level;
	
	//deltaTime_angular = tick-previousTick_angular;

	//When encoder a has a rising edge encoder B will be low if the motor is rotating clockwise
	//If the motor is rotating counter clockwise channel B will be high when channel A has a 	  
	//rising edge
	if(encoderA == 1 && encoderB == 0){
		CW = 1;
		//Adding the angle of one pulse in the direction the motor is rotating, (360/PPR = angle of one pulse)
	
		currentAngle = currentAngle + (360/PPR);
		//angularSpeed = (360/PPR)/(deltaTime_angular/1000000);
	}else if(encoderA == 1 && encoderB == 1){
		CW = 0;
		//Adding the angle of one pulse in the direction the motor is rotating
		currentAngle = currentAngle - (360/PPR);
		//angularSpeed = -(360/PPR)/(deltaTime_angular/1000000);
	}
	


	//previousTick_angular = tick;	
}

//Recieve encoder data from channel B. Enters when the level changes on the GPIO pin
void encoderInterruptB(int gpio, int level, uint32_t tick){
	encoderB = level;
	
}


//Controlling the motor through a ROS topic
void velCallback(const geometry_msgs::Twist::ConstPtr& vel) {
	vel_data  = vel->linear.x;
	ROS_INFO("velCallback");
	switch(vel_data){
	case 1:
		if(speed < 1000000){
			speed = speed+100000;
			if(speed < 0){
				gpioWrite(DIR,1);
			}else{
				gpioWrite(DIR,0);
			}
			gpioHardwarePWM(GPIO,50000,abs(speed));
			printf("Duty cycle increased to: [%f] \n",(float)gpioGetPWMdutycycle(18)/1000000);
		}
		break;
	case 2:
		speed = 0;
		gpioHardwarePWM(GPIO,50000,speed);
		printf("Duty cycle set to: [%f] \n",(float)gpioGetPWMdutycycle(18)/1000000);
		ROS_INFO("current angle: [%lf]",currentAngle);
		break;
	case 3:
		if(speed > -1000000){
			speed = speed-100000;
			if(speed < 0){
				gpioWrite(DIR,1);
			}else{
				gpioWrite(DIR,0);
			}
			gpioHardwarePWM(GPIO,50000,abs(speed));
			printf("Duty cycle decreased to: [%f] \n",(float)gpioGetPWMdutycycle(18)/1000000);
		}
		break;
	case 4:
	

		//Set speed
		speed = vel->linear.z*10000;
		gpioHardwarePWM(GPIO,50000,speed);
		
		//Negative or positive angle, i.e direction of the motor
		
		if(currentAngle < vel->linear.y){
			gpioWrite(DIR,0);
			ROS_INFO("CW");
			while(currentAngle <= vel->linear.y){	
				if(stop == 1){
					break;
				}
			}
			
		}else if(currentAngle  > vel->linear.y){
			gpioWrite(DIR,1);
			ROS_INFO("CCW");
			while(currentAngle >= vel->linear.y){
				if(stop == 1){
					break;
				}	
			}
		}

		speed = 0;
		gpioWrite(DIR,0);
		gpioHardwarePWM(GPIO,50000,0);
		
		ROS_INFO("Current angle: [%f] ",currentAngle);
		break;
	}
}


int main (int argc, char **argv)
{

//Initialise ROS node
  ros::init(argc, argv, "pwm", ros::init_options::NoSigintHandler);
  ros::NodeHandle node;
  
  geometry_msgs::Twist speed_msg;
  std_msgs::Float32 rotation_msg;
  signal (SIGINT, sigintHandler);
  ros::Rate rate(100);  // hz
  


int turn = 0;

gpioCfgClock(1,1,0);

  if (gpioInitialise() < 0)
{
   ROS_INFO("init failed\n");// pigpio initialisation failed.
   return 0;
}else
{
   // pigpio initialised okay.
   ROS_INFO("init ok\n");
}

//Set gpio mode for the needed GPIO pins
gpioSetMode(SOFTPWM,PI_OUTPUT);
gpioSetMode(GPIO,PI_OUTPUT);
//gpioSetMode(PPM,PI_INPUT);
gpioSetMode(ENCODERA,PI_INPUT);
gpioSetMode(ENCODERB,PI_INPUT);
//gpioSetMode(WALKASSIST,PI_OUTPUT);
gpioSetMode(DIR,PI_OUTPUT);
gpioSetMode(15,PI_OUTPUT);
//gpioSetMode(SPEEDOMETER,PI_INPUT);

gpioWrite(DIR,turn);
//gpioWrite(WALKASSIST,1);
//gpioSetPWMfrequency(SOFTPWM,10);

//Functions for recieving encoder data
gpioSetAlertFunc(ENCODERA, encoderInterruptA);
gpioSetAlertFunc(ENCODERB, encoderInterruptB);
//gpioSetAlertFunc(SPEEDOMETER, speedometerInterrupt);

gpioSetAlertFunc(PPM, interruptPPM);

ros::Publisher pub_speed = node.advertise<geometry_msgs::Twist>("speed_topic", 10);
ros::Publisher pub_rotation = node.advertise<std_msgs::Float32>("rotation_topic",10);

while(ros::ok()) {
	ros::Subscriber vel_sub = node.subscribe("sender_topic",10, velCallback);
	//ROS_INFO("[%lf]",currentAngle);	
	rotation_msg.data = currentAngle;
  	pub_rotation.publish(rotation_msg);

	speed_msg.linear.x = linearSpeed;
	speed_msg.angular.x = angularSpeed;

	pub_speed.publish(speed_msg);
	ros::spinOnce();
	
}
return 0;
}
