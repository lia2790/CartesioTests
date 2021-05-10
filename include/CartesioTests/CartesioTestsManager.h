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

#ifndef __CARTESIO_TESTS_MANAGER_
#define __CARTESIO_TESTS_MANAGER_

#include <CartesioTests/CartesioTestsHandler.h>

// Eigen 
#include <Eigen/Dense>
#include <Eigen/Core>

// ROS
#include <ros/ros.h>

/**
 * @brief Class Cartesio Tests Manager
 * 
 */
class CartesioTestsManager
{
	// ROS Node --------------------------------------------------------------------------------
	ros::NodeHandle _nh; /* ROSE node handle */
	ros::Timer _timer; /* ROS timer */
	double _period; /* loop period value */
	double _rate; /* loop rate */
	double _time; /* time */
	double _gravity = 9.8; /* gravity */
	// -----------------------------------------------------------------------------------------

	// CartesioTestsManager --------------------------------------------------------------------
	CartesioTestsHandler _cartesio; /* cartesio */
	// -----------------------------------------------------------------------------------------

	// Node Input Parameters -------------------------------------------------------------------
	// Cartesio -----------------------------------------------------------------------------------
	std::string _robotUrdfPath; /* robot urdf path */
	std::string _robotSrdfPath; /* robot srdf path */
	std::string _robotModelType; /* robot model type */
	std::string _taskPath; /* task definition path */
	std::string _taskName; /* task name */
	std::string _solverType; /* cartesio solver type */
	bool _robotIsFloating; /* is a floating robot (true-false) */
	// -----------------------------------------------------------------------------------------

	// Functions -------------------------------------------------------------------------------
public:
	/////////////// COSTRUCTOR ////////////////////
	/* costructor of the class */
	CartesioTestsManager( std::string ns = "" );
	/**
	 * callback of the ROS Node 
	 * @param timer: timer event for the ROS node
	 * @return void
	 */
	void timer_callback(const ros::TimerEvent& timer);
	/**
	 * spin function
	 * @return void
	 */
	void spin();

	/////////////// DISTRUCTOR ////////////////////
	/* distructor of the class */
	~CartesioTestsManager();

	private:
	/**
	 * init ROS Node
	 * @return void
	 */
	void initROSNode();
	/**
	 * load inputs
	 * @return void
	 */
	void loadInputs();
	/**
	 * init cartesio
	 * @return void
	 */
	void initCartesIO();
};

#endif // __CARTESIO_TESTS_MANAGER_
