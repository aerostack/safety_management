/*!*******************************************************************************************
 *  \file       recovery_manager_process.h
 *  \brief      Recovey manager implementation file.
 *  \details    
 *  \authors    Javier Cabrera Marugan
 *              Martín Molina
 *  \copyright  Copyright (c) 2019 Universidad Politecnica de Madrid
 *              All rights reserved
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/ 


#ifndef RECOVERY_MANAGER_PROCESS
#define RECOVERY_MANAGER_PROCESS

#include <string>
#include <robot_process.h>
#include "ros/ros.h"

#include <behavior_execution_manager_msgs/BehaviorActivationFinished.h>
#include <belief_manager_msgs/QueryBelief.h>
#include <belief_manager_msgs/RemoveBelief.h>
#include <aerostack_msgs/RecoveryPlan.h>
#include <aerostack_msgs/RecoveryPlanFinished.h>
#include <behavior_coordinator_msgs/StartTask.h>
#include <behavior_coordinator_msgs/TaskStopped.h>
#include <behavior_coordinator_msgs/TaskCommand.h>
#include <thread>
#include <pthread.h>
#include <vector> 



class RecoveryManagerProcess : public RobotProcess
{
public:
    RecoveryManagerProcess();
    ~RecoveryManagerProcess();
    double get_moduleRate();    
private: /*RobotProcess*/
    void ownSetUp();
    void ownStart();
    void ownStop();
    void ownRun();
    ros::NodeHandle n;
    double rate;
    std::string robot_namespace;
    int id;
    std::string current_plan;
    
    behavior_coordinator_msgs::TaskStopped msg_from_activation;
   
    ros::ServiceClient activate_task_srv;
    ros::ServiceServer recovery_plan_server;
    ros::ServiceClient query_client;
    ros::ServiceClient remove_client;
    ros::Publisher recovery_plan_finished;
 
//Callbacks
    bool executeRecoveryPlan(aerostack_msgs::RecoveryPlan::Request& req, aerostack_msgs::RecoveryPlan::Response& res);
    void return_starting_point();
    void emergency_land();


};

#endif 



