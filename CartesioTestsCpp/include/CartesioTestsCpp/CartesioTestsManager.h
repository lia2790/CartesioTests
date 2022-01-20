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

#include <CartesioTestsCpp/CartesioTestsHandler.h>

// Eigen 
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry> 

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

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
	ros::Subscriber _targetSubscriber; // target subscriber

	// Input Parameters ------------------------------------------------------------------------
	// Cartesio --------------------------------------------------------------------------------
	std::string _robotUrdfPath; /* robot urdf path */
	std::string _robotSrdfPath; /* robot srdf path */
	std::string _robotModelType; /* robot model type */
	std::string _taskPath; /* task definition path */
	std::string _taskName; /* task name */
	std::string _solverType; /* cartesio solver type */
	std::string _targetTopicName; /* topic name */
	bool _robotIsFloating; /* is a floating robot (true-false) */
	std::vector<std::vector<double>> _targetPositions; /* target position */
	std::vector<std::vector<double>> _targetOrientations; /* target orientation */
	std::vector<double> _targetTimes; /* target reaching time */
	int _nTargets; /* n targets */
	int _nTargetsSent = 0; /* n targets sent */
	double _homingTime; /* home reaching time */
	bool _targetSubscribed = false;
	// -----------------------------------------------------------------------------------------

	// Status Parameters -----------------------------------------------------------------------
	// -----------------------------------------------------------------------------------------
	// bool _control = true; /* control status (false: control finished - true: to be controlled) */
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
	 * list targets
	 * @return void
	 */
	void listTargets();
	/**
	 * init cartesio
	 * @return void
	 */
	void initCartesIO();
	/**
	 * robot homing
	 * @return void
	 */
	void robotHome();
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
	 * 
	 */
	void subscribeTargetInput(const geometry_msgs::PoseStamped::ConstPtr& msg);

	//
	void setOrientationFromQuaternion(double x, double y, double z, double w);
};

#endif // __CARTESIO_TESTS_MANAGER_
