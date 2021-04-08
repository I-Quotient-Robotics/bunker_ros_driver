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

#include <iostream>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <controller_manager/controller_manager.h>

#include "bunker_base/bunker_hardware.h"

typedef boost::chrono::steady_clock time_source;

void controlThread(bunker_base::BunkerHardware* robot, controller_manager::ControllerManager* cm)
{
  ros::Rate rate(50);
  time_source::time_point last_time = time_source::now();
  while (1)
  {
    // Calculate monotonic time elapsed
    time_source::time_point this_time = time_source::now();
    boost::chrono::duration<double> elapsed_duration = this_time - last_time;
    ros::Duration dt(elapsed_duration.count());
    last_time = this_time;

    robot->updateJointsFromHardware();
    cm->update(ros::Time::now(), dt);
    robot->writeCommandsToHardware();

    rate.sleep();
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "bunker_base");
  ros::NodeHandle nh, private_nh("~");

  // Initialize robot hardware and link to controller manager
  bunker_base::BunkerHardware bunker(nh, private_nh);
  controller_manager::ControllerManager cm(&bunker, nh);
  boost::thread(boost::bind(controlThread, &bunker, &cm));

  ros::spin();

  return 0;
}