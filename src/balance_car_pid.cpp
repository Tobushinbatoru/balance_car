#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "geometry_msgs/Twist.h"
#include <chrono>
#include "PID.h"

PID left_pid(2.0, 5.0, 1.0, 0.0, 255.0, 0.01);

// Float32MultiArrayメッセージを作成
std_msgs::Float32MultiArray float_array_msg;

float TORED = 0.3;
float MAX_SPEED = 1.0;
float MAX_SPIN = 2.5;

long left_trick;
long right_trick;

float left_target_speed; 
float right_target_speed;

// コールバック関数の定義
void int16ArrayCallback(const std_msgs::Int16MultiArray::ConstPtr& msg) {
    left_trick = msg->data[0];
    right_trick = msg->data[1];
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    //ROS_INFO("Linear Velocity (x, y, z): %f, %f, %f", msg->linear.x, msg->linear.y, msg->linear.z);
    //ROS_INFO("Angular Velocity (x, y, z): %f, %f, %f", msg->angular.x, msg->angular.y, msg->angular.z);
    
    const float linear_x = msg->linear.x * MAX_SPEED;
    const float angle_z = msg->angular.z * MAX_SPIN;
  	
    left_target_speed = (linear_x - 0.5 * TORED * angle_z);
    right_target_speed = (linear_x + 0.5 * TORED * angle_z);
    
    //ROS_INFO("Target Velocity (left, right): %f, %f", left_target_speed , right_target_speed);	
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "custom_node");
    ros::NodeHandle nh;

    ros::Publisher float_array_pub = nh.advertise<std_msgs::Float32MultiArray>("pid", 10);
    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, cmdVelCallback);
    ros::Subscriber sub = nh.subscribe("wheel_ticks", 10, int16ArrayCallback);

    std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
    const std::chrono::milliseconds interval(10);  // 10msのインターバル
    
    long last_left_trick = 0L;
    long last_right_trick = 0L;
    
    const float TOREAD = 0.3;
	const float pi = 3.14159265;					// pi
	const float two_pi = 6.28318531;
	const float R = 0.05;							// radius
	int8_t reducation = 54;							// gear
	int8_t pulse = 12; 								// pulse
	int per_round = pulse * reducation * 4;			// 4

    while (ros::ok()) {
        std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
        std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time);

        if (duration >= interval) {
        	int duration_ms = static_cast<int>(duration.count());
    		
            // メッセージをpublish
            float left_pulse_speed = (double)2 * pi * R * (left_trick - last_left_trick) / per_round;
            float right_pulse_speed = (double)2 * pi * R * (right_trick - last_right_trick) / per_round;
            
            last_left_trick = left_trick;
            last_right_trick = right_trick;
            
            float left_real_speed = left_pulse_speed / (static_cast<float>(duration_ms) / 1000.0f);
    		float right_real_speed = right_pulse_speed / (static_cast<float>(duration_ms) / 1000.0f);
    		
    		float left_error = left_target_speed*0.6 - left_real_speed;
    		float right_error = right_target_speed*0.6 - right_target_speed;
            
            ROS_INFO("Real Velocity (left, right): %f, %f", left_error , right_error);
            
            float_array_msg.data = {left_target_speed*0.6, right_target_speed*0.6};
            
            float_array_pub.publish(float_array_msg);
            last_time = current_time;  // 最終送信時間を更新
        }
        
        ros::spinOnce();
    }

    return 0;
}
