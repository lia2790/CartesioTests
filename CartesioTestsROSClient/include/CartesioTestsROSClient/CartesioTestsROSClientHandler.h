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

#ifndef __CARTESIO_TESTS_ROSCLIENT_HANDLER_
#define __CARTESIO_TESTS_ROSCLIENT_HANDLER_

// CartesIO
#include <cartesian_interface/ros/ROSClient.h>

// Eigen 
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace XBot::Cartesian;
/**
 * @brief Class Variable Impedance Robot
 * 
 */
class CartesioTestsROSClientHandler
{	
	// CartesioTestsROSClientHandler ------------------------------------------------------------------
	RosClient _client; /* ros client */
	CartesianTask::Ptr _task; /* cartesian task */
	double _sleepdt = 0.1; /* period */
	// ------------------------------------------------------------------------------------------------

public:
	/////////////// COSTRUCTOR ////////////////////
	/* costructor of the class */
	CartesioTestsROSClientHandler();
	/**
	 * set ros client task
	 * @param task: task name
	 * @param gain: task gain
	 * @return void
	 */
	void setROSClientTask(std::string task, double gain);
	/**
	 * set task target time
	 * @param target_time: target time
	 * @return void
	 */
	void setROSClientTaskTargetTime(double target_time);
	/**
	 * set task target time
	 * @param Ttgt: target transformation
	 * @param target_time: target time
	 * @return void
	 */
	void setROSClientTargetTask(Eigen::Affine3d Ttgt, double target_time);
	/**
	 * start control
	 * @return void
	 */
	void startControl();
	
	/////////////// DISTRUCTOR ////////////////////
	/* distructor of the class */
	~CartesioTestsROSClientHandler();
	
};

#endif // __CARTESIO_TESTS_ROSCLIENT_HANDLER_