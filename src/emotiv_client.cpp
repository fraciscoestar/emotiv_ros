#include "ros/ros.h"
#include "emotiv_ros/MentalCommand.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <stdio.h>

float maxLinearSpeed;
float maxAngularSpeed;

ros::Publisher publisher;

void MentalCommandCallback(const emotiv_ros::MentalCommand _command)
{
    geometry_msgs::Twist velocity;

    if (_command.command == "push")
    {
        velocity.linear.x = _command.strength * maxLinearSpeed;
        velocity.angular.z = 0;
    }
    else if (_command.command == "pull")
    {
        velocity.linear.x = -_command.strength * maxLinearSpeed;
        velocity.angular.z = 0;
    }
    else if (_command.command == "left")
    {
        velocity.linear.x = 0;
        velocity.angular.z = _command.strength * maxAngularSpeed;
    }
    else if (_command.command == "right")
    {
        velocity.linear.x = 0;
        velocity.angular.z = -_command.strength * maxAngularSpeed;
    }
    else
    {
        velocity.linear.x = 0;
        velocity.angular.z = 0;
    }

    publisher.publish(velocity);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "emotiv_client");
    ros::NodeHandle nh;

    nh.getParam("/max_linear_speed", maxLinearSpeed);
    nh.getParam("/max_angular_speed", maxAngularSpeed);

    ros::Subscriber commandSubscriber = nh.subscribe<emotiv_ros::MentalCommand>("mental_command", 100, MentalCommandCallback);
    publisher = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 100);

    ros::spin();
    return 0;
}