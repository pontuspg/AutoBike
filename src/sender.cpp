#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <stdlib.h> 
int main(int argc, char **argv) {

	int input;
	int deg;
	int duty;
     //Initializes ROS, and sets up a node
     ros::init(argc, argv, "sender");
     ros::NodeHandle nh;

     //Ceates the publisher, and tells it to publish
     //to the husky/cmd_vel topic, with a queue size of 100
     ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("sender_topic", 100);
	geometry_msgs::Twist msg;
	msg.linear.x = 0;

     //Sets the loop to publish at a rate of 10Hz
     ros::Rate rate(10);
	
       while(ros::ok()) {
       		printf("Enter 1 to increase speed \nEnter 2 to stop \nEnter 3 to decrease speed \nEnter 4 to select position\n ");
            scanf("%d", &input);
            switch (input){
            case 1:
            	msg.linear.x = input;
            	break;
            case 2:
            	msg.linear.x = input;
            	break;          	
            case 3:
            	msg.linear.x = input;
            	break;         
            case 4:
            	printf("Enter postition\n");
            	scanf("%d", &deg);
            	msg.linear.x = input;
            	msg.linear.y = deg;
            	printf("Enter duty cycle\n");
            	scanf("%d", &duty);
            	msg.linear.z = duty;
            	break;
            }
            pub.publish(msg);
            printf("Message sent\n\n\n\n");       
            //Reset message
            msg.linear.x = 0;
            msg.linear.y = 0;
            msg.linear.z = 0;
          //Delays untill it is time to send another message
          rate.sleep();
         }
}

