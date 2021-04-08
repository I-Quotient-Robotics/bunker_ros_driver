/*******************************************************************************
** MIT License
**
** Copyright(c) 2021 QuartzYan https://github.com/QuartzYan
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files(the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions :
**
** The above copyright notice and this permission notice shall be included in all
** copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
** OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
** SOFTWARE.
*******************************************************************************/

#ifndef BUNKER_BASE_BUNKER_HARDWARE_H
#define BUNKER_BASE_BUNKER_HARDWARE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <sensor_msgs/JointState.h>
#include <can_msgs/Frame.h>
#include <ros/ros.h>
#include <iostream>
#include <map>

#include "bunker_msgs/BunkerStatus.h"

#define WHEEL_BASE    0.555
#define WHEEL_RADIUS  0.104
#define REDUCTION_RATIO 18.0

namespace bunker_base
{
  class BunkerHardware : public hardware_interface::RobotHW
  {
  public:
    enum CanID {
      kControl = 0x111,
      kSetLight = 0x121,
      kStateFeedback = 0x211,
      kControlFeedback = 0x221,
      kLightStatus = 0x231,
      kControllerStatus = 0x241,
      kMotorStateFeedbackH1 = 0x251,
      kMotorStateFeedbackH2 = 0x252,
      kMotorStateFeedbackH3 = 0x253,
      kMotorStateFeedbackH4 = 0x254,
      kMotorStateFeedbackL1 = 0x261,
      kMotorStateFeedbackL2 = 0x262,
      kMotorStateFeedbackL3 = 0x263,
      kMotorStateFeedbackL4 = 0x264,
      kEncoder = 0x311,
      kSetCANMode = 0x421
    };

    BunkerHardware(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~BunkerHardware();

    void updateJointsFromHardware();
    void writeCommandsToHardware();

  private:
    ros::NodeHandle nh_, private_nh_;
    ros::Publisher bunker_status_pub_;
    bunker_msgs::BunkerStatus status_;

    int can_id_;
    std::string send_topic_, receive_topic_;
    ros::Publisher can_pub_;
    ros::Subscriber can_sub_;

    // ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    void EnableCANMode(bool enable);
    void CanReceiveCallback(const can_msgs::Frame::ConstPtr& msg);
    
    // Joint structure that is hooked to ros_control's InterfaceManager, 
    // to allow control via diff_drive_controller
    struct Joint
    {
      double effort;
      double position;
      double position_offset;
      double position_command;
      double velocity;
      double velocity_command;

      Joint() :
        effort(0), position(0), position_offset(0), position_command(0), velocity(0), velocity_command(0)
      { }
    } joints_[2];
  };
}  // namespace bunker_base
#endif  // BUNKER_BASE_BUNKER_HARDWARE_H