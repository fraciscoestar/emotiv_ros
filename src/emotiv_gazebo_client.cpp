#include "ros/ros.h"
#include "emotiv_ros/MentalCommand.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include <tf2/LinearMath/Quaternion.h>
#include <stdio.h>

float maxLinearSpeed;
float maxAngularSpeed;

const std::string robotName = "turtlebot";
geometry_msgs::Twist velocity;
ros::Publisher publisher;

void MentalCommandCallback(const emotiv_ros::MentalCommand _command)
{  
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
}

void GazeboPoseCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    for (int i = 0; i < msg->name.size(); i++)
    {
        if (msg->name[i] == robotName)
        {
            gazebo_msgs::ModelState state;
            state.model_name = robotName;
            state.reference_frame = "world";

            tf2::Quaternion q(msg->pose[i].orientation.x, msg->pose[i].orientation.y, msg->pose[i].orientation.z, msg->pose[i].orientation.w);

            double ang = 2*tf2::angle(q, tf2::Quaternion(0, 0, 0, 1)) + velocity.angular.z * 1/100.0;
            if (ang >= 2*M_PI)
                ang -= 2*M_PI;
            else if (ang < 0)
                ang += 2*M_PI;

            q.setRPY(0, 0, ang);   // Create this quaternion from roll/pitch/yaw (in radians)
            
            state.pose.position.x = msg->pose[i].position.x + (velocity.linear.x * 1/100.0)*cos(ang);
            state.pose.position.y = msg->pose[i].position.y + (velocity.linear.x * 1/100.0)*sin(ang);
            state.pose.position.z = msg->pose[i].position.z;

            state.pose.orientation.x = q.x();
            state.pose.orientation.y = q.y();
            state.pose.orientation.z = q.z();
            state.pose.orientation.w = q.w();

            publisher.publish(state);
        }
    }
    
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "emotiv_gazebo_client");
    ros::NodeHandle nh;

    nh.getParam("/max_linear_speed", maxLinearSpeed);
    nh.getParam("/max_angular_speed", maxAngularSpeed);

    ros::Subscriber commandSubscriber = nh.subscribe<emotiv_ros::MentalCommand>("mental_command", 100, MentalCommandCallback);
    ros::Subscriber gazeboSubscriber = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 100, GazeboPoseCallback);
    publisher = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 100);

    ros::spin();
    return 0;
}