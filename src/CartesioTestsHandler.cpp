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

#include <CartesioTests/CartesioTestsHandler.h>

CartesioTestsHandler::CartesioTestsHandler()
{

}

void CartesioTestsHandler::init(std::string URDF_PATH, std::string SRDF_PATH, bool isFloating, std::string modelType)
{
	// set the urdf and srdf path..
    _xbot_cfg.set_urdf_path(URDF_PATH);
    _xbot_cfg.set_srdf_path(SRDF_PATH);
    // the following call is needed to generate some default joint IDs
    _xbot_cfg.generate_jidmap();
    // some additional parameters..
    _xbot_cfg.set_parameter("is_model_floating_base", isFloating);
    _xbot_cfg.set_parameter<std::string>("model_type", modelType);
}

void CartesioTestsHandler::setModel()
{
    // model class
    _model = XBot::ModelInterface::getModel(_xbot_cfg);
    // homing
    Homing();
}

void CartesioTestsHandler::setProblem(std::string TASK_PATH, std::string solverType)
{
	// context object which stores some information, such as
    // the control period
	const double dt = 0.01;
    auto context = std::make_shared<Context>(std::make_shared<Parameters>(dt),_model);

    // load the ik problem given a yaml file
    auto ik_pb_yaml = YAML::LoadFile(TASK_PATH);
    ProblemDescription problem(ik_pb_yaml, context);

    // we are finally ready to make the CartesIO solver "OpenSot"
    _solver = CartesianInterfaceImpl::MakeInstance(solverType, problem, context);
}

void CartesioTestsHandler::setTask(std::string taskName)
{
    // inspect properties of "left_hand" task
    auto taskDescription = _solver->getTask(taskName);
    // check that "left_hand" is actually a Cartesian type task
    _task = std::dynamic_pointer_cast<CartesianTask>(taskDescription);
}

void CartesioTestsHandler::Homing()
{
	// initialize to a homing configuration
    Eigen::VectorXd qhome;
    _model->getRobotState("home", qhome);
    _model->setJointPosition(qhome);
    _model->update();	
}

CartesioTestsHandler::~CartesioTestsHandler()
{

}
