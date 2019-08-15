/*
test code for turn robot with action
*/
#include <ros/ros.h>
#include <string.h>
#include <cmath>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <kamerider_navigation/turn_robotAction.h>

#define PI 3.1415926
double final_goal = PI;

typedef actionlib::SimpleActionClient<kamerider_navigation::turn_robotAction> Client;

void doneCallback (const actionlib::SimpleClientGoalState& state,
                   const kamerider_navigation::turn_robotResultConstPtr& result)
{
    ROS_INFO ("I have turned %f degree, waiting for your further command", result->final_angle);
}

void activeCallback ()
{
    ROS_INFO ("Goal actived");
}

void feedbackCallback (const kamerider_navigation::turn_robotFeedbackConstPtr& feedback)
{
    ROS_INFO ("current angle is %f", feedback->current_angle);
}

int main (int argc, char** argv)
{
    ROS_INFO ("Turn robot Client online");
    ros::init (argc, argv, "turn_robot_client");

    Client client("turn_robot", true);
    ROS_INFO ("Waiting for action server to start");
    client.waitForServer();
    ROS_INFO ("Action server actived, start sending goal");

    kamerider_navigation::turn_robotGoal goal;
    goal.goal_angle = final_goal;
    client.sendGoal (goal, &doneCallback, &activeCallback, &feedbackCallback);
    ros::spin();

    return 0;
}
