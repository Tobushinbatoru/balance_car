#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#define PORT 2023

// SensorData構造体の定義
struct SensorData {
    int sensor1;
    float sensor2;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "arduino_listener");
    ros::NodeHandle nh;

    int sockfd;
    struct sockaddr_in server_addr, client_addr;

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        ROS_ERROR("Socket creation error");
        return -1;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ROS_ERROR("Bind failed");
        return -1;
    }

    ROS_INFO("Waiting for data...");

    while (ros::ok()) {
        SensorData receivedData;
        recvfrom(sockfd, &receivedData, sizeof(receivedData), 0, NULL, NULL);

        // 受信したデータをROSのトピックに発行する
        ros::Publisher sensor1_pub = nh.advertise<std_msgs::Int32>("sensor1", 10);
        ros::Publisher sensor2_pub = nh.advertise<std_msgs::Float32>("sensor2", 10);

        std_msgs::Int32 sensor1_msg;
        sensor1_msg.data = receivedData.sensor1;
        sensor1_pub.publish(sensor1_msg);

        std_msgs::Float32 sensor2_msg;
        sensor2_msg.data = receivedData.sensor2;
        sensor2_pub.publish(sensor2_msg);

        ros::spinOnce();
    }

    close(sockfd);
    return 0;
}

