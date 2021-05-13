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

#ifndef __CARTESIO_TESTS_HANDLER_
#define __CARTESIO_TESTS_HANDLER_

// CartesIO
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <thread>

// Eigen 
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace XBot::Cartesian;
/**
 * @brief Class Variable Impedance Robot
 * 
 */
class CartesioTestsHandler
{	
	// CartesioTestsHandler ------------------------------------------------------------------
	XBot::ConfigOptions _xbot_cfg; /* cartesio configuration */
	XBot::ModelInterface::Ptr _model; /* robot model */
	CartesianTask::Ptr _task; /* task */
	CartesianInterfaceImpl::Ptr _solver; /* cartesian interface solver */
	double _dt; /* period */

public:
	/////////////// COSTRUCTOR ////////////////////
	/* costructor of the class */
	CartesioTestsHandler();
	/**
	 * init robot
	 * @param URDF_PATH: robot urdf path
	 * @param SRDF_PATH: robot srdf path
	 * @param isFloating: is floating robot
	 * @param modelType: robot model type
	 * @return void
	 */
	void init(std::string URDF_PATH, std::string SRDF_PATH, bool isFloating, std::string modelType);
	/**
	 * set model
	 * @return void
	 */
	void setModel();
	/**
	 * set problem
	 * @param TASK_PATH: problem description (task) path
	 * @return void
	 */
	void setProblem(std::string TASK_PATH, std::string solverType);
	/**
	 * set task
	 * @param taskName: task name
	 * @return void
	 */
	void setTask(std::string taskName);
	/**
	 * set task target time
	 * @param target_time: target time
	 * @return void
	 */
	void setTaskTargetTime(double target_time);
	/**
	 * set task target time
	 * @param Tref: reference transformation
	 * @param target_time: target time
	 * @return void
	 */
	void setTaskTarget(Eigen::Affine3d Tref, double target_time);
	/**
	 * start control
	 * @return void
	 */
	void startControl();

	
	/////////////// DISTRUCTOR ////////////////////
	/* distructor of the class */
	~CartesioTestsHandler();

private:
	/**
	 * set home
	 * @return void
	 */
	void homing();
	/**
	 * control robot: compute the control law and update the state
	 * @return void
	 */
	void control();
	/**
	 * return joint position norm
	 * @return joint position norm
	 */
	double precision();

};

#endif // __CARTESIO_TESTS_HANDLER_