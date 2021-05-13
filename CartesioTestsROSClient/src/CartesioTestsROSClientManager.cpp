/*
 * Copyright (C) 2021 IIT-HHCM
 * Author: Liana Bertoni
 * email:  liana.bertoni@iit.it
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#include <CartesioTestsROSClient/CartesioTestsROSClientManager.h>


CartesioTestsROSClientManager::CartesioTestsROSClientManager ( std::string ns ) : _nh ( ns )
{
	//initialization ROS node
	ROS_INFO_STREAM ("I am initializing the Node...");
	initROSNode();

	//initialization ROS node
	ROS_INFO_STREAM ("I am loading the Node Inputs...");
	loadInputs();

	//initialization ROS node
	ROS_INFO_STREAM ("I am initializing CartesIO ROSClient...");
	initCartesIOROSClientROSClient();

	//initialization done
	ROS_INFO_STREAM ("Initialization done.");
}

void CartesioTestsROSClientManager::initROSNode()
{
	// init ROS node
	if (! _nh.getParam("/rate", _rate))
        _rate = 100.0;
    _period = 1.0 / _rate;
    _timer = _nh.createTimer(ros::Duration(_period), &CartesioTestsROSClientManager::timer_callback, this, false, false);
    _time = 0.0;
}
void CartesioTestsROSClientManager::loadInputs()
{
	// loading inputs
	_nh.param<std::string>("/task_name",_taskName,"taskA");
	_nh.param<double>("/task_gain",_taskGain,0.5);
	_nh.param<double>("/target_time",_targetTime,0.03);
}

void CartesioTestsROSClientManager::initCartesIOROSClient()
{
	// init cartesio
	_cartesioROSClient.setROSClientTask(_taskName,_taskGain);
	_cartesioROSClient.setROSClientTaskTargetTime(_targetTime);
	_cartesioROSClient.startControl();
}

bool CartesioTestsROSClientManager::newReference()
{
	// new reference
	return false;
}

void CartesioTestsROSClientManager::startControl()
{
	// start control
	_cartesioROSClient.startControl();
}

void CartesioTestsROSClientManager::timer_callback(const ros::TimerEvent& timer)
{
	// control
	if(newReference())
		startControl();

	_time += _period; // update time
}

void CartesioTestsROSClientManager::spin()
{
	_timer.start();
	ROS_INFO_STREAM("Cartesio Tests started looping time " << 1./_period << "Hz");
	ros::spin();
}

CartesioTestsROSClientManager::~CartesioTestsROSClientManager()
{

}
