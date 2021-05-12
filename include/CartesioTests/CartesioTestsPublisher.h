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

#ifndef __CARTESIO_TESTS_PUBLISHER_
#define __CARTESIO_TESTS_PUBLISHER_

// Eigen 
#include <Eigen/Dense>
#include <Eigen/Core>

// ROS
#include <ros/ros.h>
#include <robot_state_publisher/robot_state_publisher.h>

class CartesioTestsPublisher
{
	// ROS Node --------------------------------------------------------------------------------
	ros::NodeHandle _nh; /* ROSE node handle */
	ros::Publisher joints_pub; /* joints state publisher */

public:
	/////////////// COSTRUCTOR ////////////////////
	/* costructor of the class */
	CartesioTestsPublisher();

	/**
	 * init publisher
	 * @return void
	 */
	void init();

	/////////////// DISTRUCTOR ////////////////////
	/* distructor of the class */
	~CartesioTestsPublisher();

private:

};

#endif // __CARTESIO_TESTS_HANDLER_