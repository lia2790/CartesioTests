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

#ifndef __CARTESIO_TESTS_ROSCLIENT_MANAGER_
#define __CARTESIO_TESTS_ROSCLIENT_MANAGER_

#include <CartesioTestsROSClient/CartesioTestsROSClientHandler.h>

// Eigen 
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

// ROS
#include <ros/ros.h>

/**
 * @brief Class Cartesio Tests ROS Client Manager
 * 
 */
class CartesioTestsROSClientManager
{
	// ROS Node --------------------------------------------------------------------------------
	ros::NodeHandle _nh; /* ROSE node handle */
	ros::Timer _timer; /* ROS timer */
	double _period; /* loop period value */
	double _rate; /* loop rate */
	double _time; /* time */
	double _gravity = 9.8; /* gravity */
	// -----------------------------------------------------------------------------------------

	// CartesioTestsROSClientManager --------------------------------------------------------------------
	CartesioTestsROSClientHandler _cartesioROSClient; /* cartesio ros client*/
	// -----------------------------------------------------------------------------------------

	// Input Parameters ------------------------------------------------------------------------
	// Cartesio --------------------------------------------------------------------------------
	std::string _taskName; /* task name */
	std::vector<double> _taskTrasl; /* task target translation */
	std::vector<double> _taskOrient; /* task target orientation */
	Eigen::Affine3d _taskTarget; /* task target transformation matrix */
	double _taskGain; /* task gain */
	double _targetTime; /* target reaching time */
	// -----------------------------------------------------------------------------------------

	public:
	/////////////// COSTRUCTOR ////////////////////
	/* costructor of the class */
	CartesioTestsROSClientManager( std::string ns = "" );
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
	~CartesioTestsROSClientManager();

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
	void initCartesIOROSClient();
	/**
	 * set new task reference
	 * @return true if a new reference received otherwise false
	 */
	bool newReference();
	/**
	 * start control
	 * @return void
	 */
	void startControl();
	/**
	 * minimal representation to transform
	 * @param taskTrasl: task translation
	 * @param taskOrient: task orientation
	 * @param taskTarget: returned task target
	 * @return void
	 */
	void minimalRepresToTransform(std::vector<double> taskTrasl, std::vector<double> taskOrient, Eigen::Affine3d& taskTarget);
};

#endif // __CARTESIO_TESTS_ROSCLIENT_MANAGER_
