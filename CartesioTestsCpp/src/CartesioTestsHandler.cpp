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

#include <CartesioTestsCpp/CartesioTestsHandler.h>

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
    // default init
    _dt = 0.01;
    _target = Eigen::Affine3d::Identity();
    _targetTime = 1.0;
}

void CartesioTestsHandler::setModel()
{
    // set model
    _model = XBot::ModelInterface::getModel(_xbot_cfg);
    // home state
    homing();
}

void CartesioTestsHandler::setRobot()
{
    // returns a shared pointer to a robot object
    _robot = XBot::RobotInterface::getRobot(XBot::Utils::getXBotConfig());
}

void CartesioTestsHandler::setProblem(std::string TASK_PATH, std::string solverType)
{
	// context object which stores some information, such as
    auto context = std::make_shared<Context>(std::make_shared<Parameters>(_dt),_model);

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

void CartesioTestsHandler::setTaskTargetTime(double target_time)
{
    // set reference
    Eigen::Affine3d Tref;
    _task->getPoseReference(Tref);
    Tref.translation()[1] += 0.05;
    setTaskTarget(Tref, target_time);
}

void CartesioTestsHandler::setTarget(std::vector<double> targetPosition, std::vector<double> targetOrientation, double target_time)
{
    minToTrasform(targetPosition,targetOrientation);
    setTaskTarget(_target,target_time);
}

void CartesioTestsHandler::setTaskTarget(Eigen::Affine3d Tref, double target_time)
{
    // set task target
    _target = Tref;
    _targetTime = target_time;
}

void CartesioTestsHandler::control()
{
    // control law robot
    Eigen::VectorXd q, qdot, qddot;
    _model->getJointPosition(q);
    _model->getJointVelocity(qdot);
    _model->getJointAcceleration(qddot);

    q += _dt * qdot + 0.5 * std::pow(_dt, 2) * qddot;
    qdot += _dt * qddot;

    _model->setJointPosition(q);
    _model->setJointVelocity(qdot);
    _model->update();

    // move robot
    Eigen::VectorXd qr, qdotr;
    _robot->setReferenceFrom(*_model);
    _robot->getPositionReference(qr);
    _robot->getVelocityReference(qdotr);
        
    _robot->setPositionReference(qr);
    _robot->setVelocityReference(qdotr);
    _robot->move();
}

double CartesioTestsHandler::precision()
{
    Eigen::VectorXd qdot;
    _model->getJointVelocity(qdot);
    return qdot.norm();
}

void CartesioTestsHandler::goHome(double time_to_home)
{
    // set home
    setHome(time_to_home);
    // go home
    startControl();
}

void CartesioTestsHandler::reachTarget(int& current_state, bool& yes)
{
    // reach target routine
    if(current_state == 0) // here we command a reaching motion
    {
        std::cout << "Commanding..." << std::endl;
        _task->setPoseTarget(_target, _targetTime);
        current_state++;
        yes = false;
    }
    if(current_state == 1)
    {   
        if(_task->getTaskState() == State::Reaching)
        {
            std::cout << "Motion started!" << std::endl;
            current_state++;
            yes = false;
        }
    }
    if(current_state == 2) // here we wait for it to be completed
    {
        if(_task->getTaskState() == State::Online)
        {
            Eigen::Affine3d T;
            _task->getCurrentPose(T);
            std::cout << "Motion completed!" << "Final error " 
                    <<(T.inverse()*_target).translation().norm() << std::endl;
            std::cout << "Motion completed!" << "Final Position " 
                    << T.translation().transpose() << std::endl;
            current_state++;
            yes = false;
        }
    }
    if(current_state == 3) // here we wait the robot to come to a stop
    {
        std::cout << "qdot norm is " << precision() << std::endl;
        if(precision() < 1e-3)
        {
            std::cout << "Robot came to a stop, press ENTER to continue commanding.. \n";
            std::cin.ignore();
            current_state++;
            yes = false;
        }
    }
    if(current_state == 4) yes = true;
}

void CartesioTestsHandler::startControl()
{
    int current_state = 0;
    double time = 0;
    bool yes = false;
    while(true)
    {
        // reaching target
        reachTarget(current_state,yes);
        if(yes) break;

        // update and integrate model state
        _solver->update(time, _dt);

        // control
        control();        

        // loop at 100 Hz
        std::this_thread::sleep_for(std::chrono::duration<double>(_dt));
        time += _dt;
    }
}

void CartesioTestsHandler::homing()
{
	// initialize to a homing configuration
    Eigen::VectorXd qhome;
    _model->getRobotState("home", qhome);
    _model->setJointPosition(qhome);
    _model->update();
}

void CartesioTestsHandler::setHome(double time_to_home)
{
    // set home
    Eigen::Affine3d Tref;
    _task->getPoseReference(Tref);
    setTaskTarget(Tref,time_to_home);
}

void CartesioTestsHandler::minToTrasform(std::vector<double> targetPosition, std::vector<double> targetOrientation)
{
    // set translation
    _task->getPoseReference(_target);
    _target.translation()[0] += targetPosition[0];
    _target.translation()[1] += targetPosition[1];
    _target.translation()[2] += targetPosition[2];
    // set orientation
    targetOrientation[0] = targetOrientation[0]*(M_PI/180);
    targetOrientation[1] = targetOrientation[1]*(M_PI/180);
    targetOrientation[2] = targetOrientation[2]*(M_PI/180);
    Eigen::Quaternion<double> q = Eigen::AngleAxisd(targetOrientation[2],Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(targetOrientation[1],Eigen::Vector3d::UnitY())*
         Eigen::AngleAxisd(targetOrientation[0],Eigen::Vector3d::UnitX()); 
    _target.linear() = _target.linear()*q.toRotationMatrix();
}

CartesioTestsHandler::~CartesioTestsHandler()
{

}
