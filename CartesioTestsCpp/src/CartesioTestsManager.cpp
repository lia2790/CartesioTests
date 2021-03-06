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

#include <CartesioTestsCpp/CartesioTestsManager.h>


CartesioTestsManager::CartesioTestsManager ( std::string ns ) : _nh ( ns )
{
	//initialization ROS node
	ROS_INFO_STREAM ("I am initializing the Node...");
	initROSNode();

	//initialization ROS node
	ROS_INFO_STREAM ("I am loading the Node Inputs...");
	loadInputs();

	//initialization ROS node
	ROS_INFO_STREAM ("I am initializing CartesIO...");
	initCartesIO();

	//initialization ROS node
	ROS_INFO_STREAM ("I am configuring ROBOT HOME POSITION...");
	robotHome();

	//initialization done
	ROS_INFO_STREAM ("Initialization done.");
}

void CartesioTestsManager::initROSNode()
{
	// init ROS node
	if (! _nh.getParam("/rate", _rate))
        _rate = 100.0;
    _period = 1.0 / _rate;
    _timer = _nh.createTimer(ros::Duration(_period), &CartesioTestsManager::timer_callback, this, false, false);
    _time = 0.0;
}

void CartesioTestsManager::loadInputs()
{
	// loading inputs
	_nh.param<std::string>("/robot_urdf_path",_robotUrdfPath,"/config");
	_nh.param<std::string>("/robot_srdf_path",_robotSrdfPath,"/config");
	_nh.param<std::string>("/robot_task_path",_taskPath,"/config");
	_nh.param<std::string>("/robot_model_type",_robotModelType,"RBDL");
	_nh.param<std::string>("/solver_type",_solverType,"OpenSoT");
	_nh.param<std::string>("/task_name",_taskName,"taskA");
	_nh.param<std::vector<double>>("/target_time",_targetTimes,{1.0});
	_nh.param<double>("/homing_time",_homingTime,1.0);
	_nh.param<bool>("/robot_is_floating",_robotIsFloating,true);
	listTargets();
}

void CartesioTestsManager::initCartesIO()
{
	// init cartesio (settings-mandatory order)
	_cartesio.init(_robotUrdfPath,_robotSrdfPath,_robotIsFloating,_robotModelType);
	ROS_INFO_STREAM ("Init CartesIO...");
	_cartesio.setModel();
	ROS_INFO_STREAM ("Set Model...");
	_cartesio.setRobot();
	ROS_INFO_STREAM ("Set Robot...");
	_cartesio.setProblem(_taskPath,_solverType);
	ROS_INFO_STREAM ("Set Problem...");
	_cartesio.setTask(_taskName);
	ROS_INFO_STREAM ("Set Task...");
}

void CartesioTestsManager::robotHome()
{
	_cartesio.goHome(_homingTime);
}

bool CartesioTestsManager::newReference()
{
	// is target already sent?
	if(_nTargetsSent < _nTargets) // yes
	{
		int iTarget = _nTargetsSent;
		std::cout << "New reference" << std::endl;
		std::cout << "Pos: " << _targetPositions[iTarget][0] << ' ' << _targetPositions[iTarget][1] << ' ' << _targetPositions[iTarget][2] << std::endl;
		std::cout << "Orient: " << _targetOrientations[iTarget][0] << ' ' << _targetOrientations[iTarget][1] << ' ' << _targetOrientations[iTarget][2] << std::endl;
		_cartesio.setTarget(_targetPositions[iTarget],_targetOrientations[iTarget],_targetTimes[iTarget]);
		_nTargetsSent++;
		return true;
	}
	else // no
	{
		std::cout << "No new reference" << std::endl;
		return false;
	}
}

void CartesioTestsManager::startControl()
{
	// start control
	_cartesio.startControl();
}

void CartesioTestsManager::timer_callback(const ros::TimerEvent& timer)
{
	// control
	if(newReference())
		startControl();

	// update time
	_time += _period;
}

void CartesioTestsManager::listTargets()
{
	// list targets
	std::vector<double> targetPositions;
	std::vector<double> targetOrientations;
	_nh.param<std::vector<double>>("/task_position",targetPositions,{0.0,0.0,0.0});
	_nh.param<std::vector<double>>("/task_orientation",targetOrientations,{0.0,0.0,0.0});
	// inserting
	_nTargets = targetPositions.size()/3;
	for (int i = 0; i < targetPositions.size(); i+=3)
	{
		std::vector<double> app;
		app.push_back(targetPositions[i]);
		app.push_back(targetPositions[i+1]);
		app.push_back(targetPositions[i+2]);
		_targetPositions.push_back(app);
		app.clear();
		app.push_back(targetOrientations[i]);
		app.push_back(targetOrientations[i+1]);
		app.push_back(targetOrientations[i+2]);
		_targetOrientations.push_back(app);
	}
	// print result
	std::cout << "Targets listed: " << std::endl; 
	for (int i = 0; i < _targetPositions.size(); ++i)
		std::cout << _targetPositions[i][0] << ' ' << _targetPositions[i][1] << ' ' << _targetPositions[i][2] << ' ';
	std::cout << std::endl;
	for (int i = 0; i < _targetOrientations.size(); ++i)
		std::cout << _targetOrientations[i][0] << ' ' << _targetOrientations[i][1] << ' ' << _targetOrientations[i][2] << ' ';
	std::cout << std::endl;
}

void CartesioTestsManager::spin()
{
	_timer.start();
	ROS_INFO_STREAM("Cartesio Tests started looping time " << 1./_period << "Hz");
	ros::spin();
}

CartesioTestsManager::~CartesioTestsManager()
{

}
