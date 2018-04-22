/*
* Software License Agreement (BSD License)
* Copyright (c) 2013, Georgia Institute of Technology
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**********************************************
 * @file ConstantSpeedController.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date April 14, 2014
 * @copyright 2014 Georgia Institute of Technology
 * @brief Controller to drive robot at constant speed
 *cycle
 * @details ComstantSpeed Controller class implementation
 ***********************************************/

#include <math.h>
#include <pluginlib/class_list_macros.h>
#include "ConstantSpeedController.h"
#include "autorally_msgs/runstop.h"
#include <sensor_msgs/JointState.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <gazebo_msgs/ModelStates.h>


#define PI 3.14159265
#define DEGTORAD (PI/180)

PLUGINLIB_DECLARE_CLASS(autorally_control, ConstantSpeedController, autorally_control::ConstantSpeedController, nodelet::Nodelet)

namespace autorally_control
{

ConstantSpeedController::ConstantSpeedController():
  m_constantSpeedPrevThrot(0.0),
  m_integralError(0.0)
{}

ConstantSpeedController::~ConstantSpeedController()
{}

void ConstantSpeedController::onInit()
{
  NODELET_INFO("ConstantSpeedController initialization");
  max_delta_v_per_second = 0.5;
  period_in_second = 0.01;//YT maybe not true, but it will be adjusted through cycle
  max_turning_angle_degree = 20;
  max_steering_acc_degree = 5;
  max_delta_v_per_second = 0.5;//YT m/s
  throttle_acceleration_map_.max_throttle = 0.3;
  throttle_acceleration_map_.min_throttle = 0;
  throttle_acceleration_map_.throttle_factor = 1;
  wheelbase = 0.57;
  robotvelfromwheel_buf.data = 0;
  robotvelfromwheel_buf_buf.data = 0;
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle nhPvt = getPrivateNodeHandle();
  overtime = new ros::Duration(0.5);//500ms
  overtime_last = ros::Time::now();
  overtime_current = ros::Time::now();

  m_mostRecentSpeedCommand.drive.speed = 0;

  m_speedCommandSub = nh.subscribe("/cmd_vel", 1, &ConstantSpeedController::cmdvelCallback, this);
  m_wheelSpeedsSub = nh.subscribe("wheelSpeeds", 1,
                          &ConstantSpeedController::wheelSpeedsCallback,
                          this);
  yt_jointstateSub = nh.subscribe("/autorally_platform/joint_states", 1, &ConstantSpeedController::ytjointstateCallback, this);
  yt_move_base_runstopSub = nh.subscribe("/yt_move_base_runstop", 1, &ConstantSpeedController::ytrunstopCallback, this);


  m_chassisCommandPub = nh.advertise<autorally_msgs::chassisCommand>
                        ("constantSpeedController/chassisCommand", 1);
  yt_robotvelfromwheelPub = nh.advertise<std_msgs::Float64>("constantSpeedController/yt_robotvelfromwheel", 1);
  yt_robotsteerPub = nh.advertise<std_msgs::Float64>("constantSpeedController/yt_robotsteer", 1);
  yt_robotsteersetpointPub = nh.advertise<std_msgs::Float64>("constantSpeedController/robotsteersetpoint", 1);
  if(!nhPvt.getParam("KP", m_constantSpeedKP) ||
     !nhPvt.getParam("KD", m_constantSpeedKD) ||
     !nhPvt.getParam("KI", m_constantSpeedKI) ||
     !nhPvt.getParam("IMax", m_constantSpeedIMax))
  {
    NODELET_ERROR("Could not get all ConstantSpeedController params");
  }

  NODELET_INFO("ConstantSpeedController initialization complete");

}

void ConstantSpeedController::cmdvelCallback(const geometry_msgs::Twist& msg)
{
    overtime_last = ros::Time::now();
    m_mostRecentSpeedCommand.drive.steering_angle = -std::atan(msg.angular.z*wheelbase/msg.linear.x);//rad
    m_mostRecentSpeedCommand.drive.speed = msg.linear.x;
}

void ConstantSpeedController::wheelSpeedsCallback(const autorally_msgs::wheelSpeedsConstPtr& msg)
{
    static int i = 0;

    m_currentrobotstate.drive.speed = 0.5*(msg->lbSpeed + msg->rbSpeed);

    std_msgs::Float64 temp;

    //YT output the robot velocity with moving average method
    temp.data = (m_currentrobotstate.drive.speed + robotvelfromwheel_buf.data + robotvelfromwheel_buf_buf.data)/3;
    yt_robotvelfromwheelPub.publish(temp);
    robotvelfromwheel_buf_buf.data = robotvelfromwheel_buf.data;
    robotvelfromwheel_buf.data = temp.data;

    autorally_msgs::chassisCommandPtr command(new autorally_msgs::chassisCommand);
    command->header.stamp = ros::Time::now();
    command->header.frame_id = "joystick";
    command->sender = "joystick";

    /// YT
    /// command->steering:left(-1), right(+1)
    /// m_mostRecentSpeedCommand: left(-maxrad), right(+maxrad)
    /// m_steering_rad: left(+currentrad), right(-currentrad), should be toggled
    /// but the function speedCallback() can only calculate the steering angle of front wheel,
    /// so we must saturate the steering angle

    temp.data = m_currentrobotstate.drive.steering_angle;
    yt_robotsteerPub.publish(temp);

    steeringCB(command);

    throttleCB(command);

    overtimeCB(command);

    ///YT divide the publish frequency
    i++;
    if(i > 4)i -= 3;
    if( yt_publish_enabled && (i % 3 == 0))
    {
        m_chassisCommandPub.publish(command);



        //YT refresh the observation of control period
        period_in_second = (ros::Duration(ros::Time::now() - calculate_period_last)).toSec();
        calculate_period_last = ros::Time::now();
    }

    ///YT show the setpoint
    temp.data = command->steering * 0.348;
    yt_robotsteersetpointPub.publish(temp);
}

void ConstantSpeedController::ytjointstateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    ///YT
    /// get steer from modelstate topic
    /// since the simulated car is not purily ackermann
    //YT: we should toggle the sign here
    m_currentrobotstate.drive.steering_angle = -(msg->position.at(4) + msg->position.at(9))/2;
}

void ConstantSpeedController::ytrunstopCallback(const autorally_msgs::runstop::ConstPtr& msg)
{
    yt_publish_enabled = msg->motionEnabled;
}

void ConstantSpeedController::throttleCB(autorally_msgs::chassisCommandPtr command)
{
    command->frontBrake = 0.0;
    double delta_v_between_command_and_state = m_mostRecentSpeedCommand.drive.speed - m_currentrobotstate.drive.speed;

    double acc;

    //saturate the acceleration
    if( delta_v_between_command_and_state > (max_delta_v_per_second * period_in_second) )
    {
        m_currentCommandSetPoint.drive.speed = m_currentrobotstate.drive.speed + max_delta_v_per_second * period_in_second;
        acc = max_delta_v_per_second * period_in_second;
    }
    else if( delta_v_between_command_and_state < -(max_delta_v_per_second * period_in_second) )
    {
        m_currentCommandSetPoint.drive.speed = m_currentrobotstate.drive.speed - max_delta_v_per_second * period_in_second;
        acc = -max_delta_v_per_second * period_in_second;
    }
    else
    {
        m_currentCommandSetPoint.drive.speed = m_mostRecentSpeedCommand.drive.speed;
        acc = delta_v_between_command_and_state;
    }

    ///YT the throttle describes the acceleration, so we need to trace throttle based on delta_v
    calculateThrottle(command, acc);
}


void ConstantSpeedController::steeringCB(autorally_msgs::chassisCommandPtr command)
{
    if((m_mostRecentSpeedCommand.drive.steering_angle - m_currentrobotstate.drive.steering_angle) * 180/PI > max_steering_acc_degree )//should turn right more
    {
        //NODELET_INFO_STREAM("YT: should turn right more");
        command->steering = m_currentrobotstate.drive.steering_angle *180/PI + max_steering_acc_degree;
    }
     else if ((m_mostRecentSpeedCommand.drive.steering_angle - m_currentrobotstate.drive.steering_angle) * 180/PI < -max_steering_acc_degree )//should turn left more
    {
        //NODELET_INFO_STREAM("YT: should turn left more");
         command->steering = m_currentrobotstate.drive.steering_angle *180/PI - max_steering_acc_degree;
    }
    else
    {
        command->steering = m_mostRecentSpeedCommand.drive.steering_angle *180/PI;
    }

    if(fabs( command->steering )> 20)
        command->steering = command->steering / fabs( command->steering );
    else
        command->steering = command->steering / 20;
}

void ConstantSpeedController::overtimeCB(autorally_msgs::chassisCommandPtr command)
{
    ///YT stop moving if we cannot get command
    overtime_current = ros::Time::now();
    if((overtime_current.toSec() - overtime_last.toSec() )> overtime->toSec())
    {
        command->steering = 0;
        command->throttle = 0;
        command->frontBrake = 1;
    }
}

void ConstantSpeedController::calculateThrottle(autorally_msgs::chassisCommandPtr command, double acc)
{
    double p = throttle_acceleration_map_.getThrottle(acc);
    if(yt_publish_enabled)
    {
        m_integralError += m_currentCommandSetPoint.drive.speed - m_currentrobotstate.drive.speed;
        if (m_integralError > (m_constantSpeedIMax / m_constantSpeedKI))
        {
            m_integralError = (m_constantSpeedIMax / m_constantSpeedKI);
        }
        if (m_integralError < -(m_constantSpeedIMax / m_constantSpeedKI))
        {
            m_integralError = -(m_constantSpeedIMax / m_constantSpeedKI);
        }
        command->throttle =
                 p + m_constantSpeedKP*(m_currentCommandSetPoint.drive.speed - m_currentrobotstate.drive.speed);
        command->throttle += m_constantSpeedKI * m_integralError;
        command->throttle = std::max(-1.0, std::min(1.0, command->throttle));
    }
}

double ThrottleAccelerationMap::getThrottle(double acc)
{
    return throttle_factor * acc;
}

double ThrottleAccelerationMap::setThrottle()
{

}

}//namespace
