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

#include "bunker_base/bunker_hardware.h"
#include <boost/assign.hpp>
#include <thread>

namespace bunker_base
{
  BunkerHardware::BunkerHardware(ros::NodeHandle nh, ros::NodeHandle private_nh)
      : nh_(nh), private_nh_(private_nh)
  {
    ros::V_string joint_names = boost::assign::list_of("left_wheel_joint")("right_wheel_joint");

    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
      hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                              &joints_[i].position,
                                                              &joints_[i].velocity,
                                                              &joints_[i].effort);
      joint_state_interface_.registerHandle(joint_state_handle);

      hardware_interface::JointHandle joint_handle(joint_state_handle,
                                                   &joints_[i].velocity_command);
      velocity_joint_interface_.registerHandle(joint_handle);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);

    bunker_status_pub_ = nh_.advertise<bunker_msgs::BunkerStatus>("bunker_status", 1);

    private_nh_.param<int>("can_id", can_id_, 1);
    private_nh_.param<std::string>("can_send_topic", send_topic_, "sent_messages");
    private_nh_.param<std::string>("can_receive_topic", receive_topic_, "received_messages");
    can_pub_ = nh_.advertise<can_msgs::Frame>(send_topic_, 1000);
    can_sub_ = nh_.subscribe<can_msgs::Frame>(receive_topic_, 10, &BunkerHardware::CanReceiveCallback, this);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    EnableCANMode(true);
  }

  BunkerHardware::~BunkerHardware()
  {
  }

  // Pull latest speed and travel measurements from MCU, and store in joint structure for ros_control
  void BunkerHardware::updateJointsFromHardware()
  {
    static ros::Time now_time;
    static ros::Time last_time;
    now_time = ros::Time::now();
    if (last_time.isZero())
    {
      last_time = now_time;
      return;
    }
    double dt = (now_time - last_time).toSec();
    if (dt == 0)
      return;
    last_time = now_time;

    joints_[0].velocity = (status_.motor_states[1].rpm / (60.0 * REDUCTION_RATIO)) * 2 * 3.1415926; //left_wheel
    joints_[1].velocity = (status_.motor_states[0].rpm / (60.0 * REDUCTION_RATIO)) * 2 * 3.1415926; //right_wheel

    joints_[0].position += joints_[0].velocity * dt;
    joints_[1].position += joints_[1].velocity * dt;
  }

  // Get latest velocity commands from ros_control via joint structure, and send to MCU
  void BunkerHardware::writeCommandsToHardware()
  {
    float vx = (joints_[0].velocity_command + joints_[1].velocity_command) * WHEEL_RADIUS / 2.0;
    float vth = (joints_[1].velocity_command - joints_[0].velocity_command) * WHEEL_RADIUS / WHEEL_BASE;
    //std::cout << "vx:" << vx << "/n" << "vth:" << vth << std::endl;

    static int16_t temp;
    static can_msgs::Frame frame;

    frame.header.stamp = ros::Time::now();
    frame.id = kControl;
    frame.is_rtr = 0;
    frame.is_extended = 0;
    frame.is_error = 0;
    frame.dlc = 8;

    temp = static_cast<int16_t>(vx * 1000.0);
    frame.data[0] = temp >> 8;
    frame.data[1] = temp;

    temp = static_cast<int16_t>(vth * 1000.0);
    frame.data[2] = temp >> 8;
    frame.data[3] = temp;

    can_pub_.publish(frame);
  }

  void BunkerHardware::EnableCANMode(bool enable)
  {
    can_msgs::Frame frame;

    frame.header.stamp = ros::Time::now();
    frame.id = kSetCANMode;
    frame.is_rtr = 0;
    frame.is_extended = 0;
    frame.is_error = 0;
    frame.dlc = 1;

    frame.data[0] = enable ? 0x01 : 0x00;

    can_pub_.publish(frame);
  }

  void BunkerHardware::CanReceiveCallback(const can_msgs::Frame::ConstPtr &msg)
  {
    int16_t temp;
    uint8_t motor_id;

    switch (msg->id)
    {
    case kStateFeedback:
      status_.base_state = msg->data[0];
      status_.control_mode = msg->data[1];
      temp = msg->data[3] | msg->data[2] << 8;
      status_.battery_voltage = temp / 10.0;
      status_.fault_code = msg->data[5];
      break;
    case kControlFeedback:
      temp = msg->data[1] | msg->data[0] << 8;
      status_.vx = temp / 1000.0;
      temp = msg->data[3] | msg->data[2] << 8;
      status_.vth = temp / 1000.0;
      break;

    case kLightStatus:
      break;
    case kControllerStatus:
      break;

    case kMotorStateFeedbackH1:
      status_.motor_states[0].id = 1;
      status_.motor_states[0].rpm = -float(int16_t(msg->data[1] | msg->data[0] << 8));
      break;
    case kMotorStateFeedbackH2:
      status_.motor_states[1].id = 2;
      status_.motor_states[1].rpm = -float(int16_t(msg->data[1] | msg->data[0] << 8));
      break;
    case kMotorStateFeedbackH3:
      break;
    case kMotorStateFeedbackH4:
      break;

    case kMotorStateFeedbackL1:
      status_.driver_states[0].driver_voltage = status_.battery_voltage;
      status_.driver_states[0].driver_temperature = float(msg->data[3] | msg->data[2] << 8);
      status_.driver_states[0].driver_state = msg->data[5];
      break;
    case kMotorStateFeedbackL2:
      status_.driver_states[1].driver_voltage = status_.battery_voltage;
      status_.driver_states[1].driver_temperature = float(msg->data[3] | msg->data[2] << 8);
      status_.driver_states[1].driver_state = msg->data[5];
      break;
    case kMotorStateFeedbackL3:
      break;
    case kMotorStateFeedbackL4:
      break;

    case kEncoder:
      status_.motor_states[0].motor_pose = double((msg->data[3] | msg->data[2] << 8 | msg->data[1] << 16 | msg->data[0] << 24)/1000.0);
      status_.motor_states[1].motor_pose = double((msg->data[7] | msg->data[6] << 8 | msg->data[5] << 16 | msg->data[4] << 24)/1000.0);
      break;
    default:
      ROS_WARN_STREAM("Unknown ID: " << std::hex << "0x" << msg->id);
    }

    bunker_status_pub_.publish(status_);
  }

} // namespace bunker_base
