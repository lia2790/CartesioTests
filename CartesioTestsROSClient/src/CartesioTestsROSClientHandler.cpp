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

#include <CartesioTestsROSClient/CartesioTestsROSClientHandler.h>

CartesioTestsROSClientHandler::CartesioTestsROSClientHandler()
{

}

void CartesioTestsROSClientHandler::setROSClientTask(std::string task, double gain)
{
	// ..get task
    auto clientTask = _client.getTask(task);
    // ..set task gain
    clientTask->setLambda(gain);
    // ..get cartesian task
     _task = std::dynamic_pointer_cast<CartesianTask>(clientTask);
}

void CartesioTestsROSClientHandler::setROSClientTaskTarget(Eigen::Affine3d Ttgt, double target_time)
{
    // set task target
    if(_task)
    {
        _task->setPoseTarget(Ttgt, target_time);
    }
}

void CartesioTestsROSClientHandler::setROSClientTaskTargetTime(double target_time)
{
    // set task target without to specify a reference
    if(_task)
    {
        Eigen::Affine3d Ttgt;
        _task->getPoseReference(Ttgt);
        _task->setPoseTarget(Ttgt, target_time);
    }
}

void CartesioTestsROSClientHandler::startControl()
{
    // start control
    // ..wait until motion started
    while(_task->getTaskState() == State::Online)
    {
        _client.update(0,0);
        ros::Duration(_sleepdt).sleep();
    }

    std::cout << "Motion Started!" << std::endl;

    // ..wait until motion completed
    while(_task->getTaskState() == State::Reaching)
    {
        _client.update(0,0);
        ros::Duration(_sleepdt).sleep();
    }
    
    std::cout << "Motion Completed!" << std::endl;   
}

CartesioTestsROSClientHandler::~CartesioTestsROSClientHandler()
{

}
