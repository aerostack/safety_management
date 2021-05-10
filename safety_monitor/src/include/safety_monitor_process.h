/*!*******************************************************************************************
 *  \file       emergency_manager_process.h
 *  \brief      Emergency manager implementation file.
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


#ifndef SAFETY_MONITOR_PROCESS
#define SAFETY_MONITOR_PROCESS

#include <string>
#include <robot_process.h>
#include "ros/ros.h"
#include <aerostack_msgs/StringStamped.h>
#include <behavior_execution_manager_msgs/BehaviorActivationFinished.h>
#include <aerostack_msgs/RecoveryPlan.h>
#include <aerostack_msgs/RecoveryPlanFinished.h>
#include <belief_manager_msgs/QueryBelief.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <map>
#include <utility> 
#include <algorithm>
#include <thread>


class SafetyMonitorProcess : public RobotProcess
{
public:
    SafetyMonitorProcess();
    ~SafetyMonitorProcess();

    bool isStarted();
    double get_moduleRate();  
protected:
    bool resetValues();    
private: /*RobotProcess*/
    void ownSetUp();
    void ownStart();
    void ownStop();
    void ownRun();
    ros::NodeHandle n;
    YAML::Node safety_rules;
    double rate;
    std::string robot_namespace;
    std::string safety_monitor_language_path;
    std::string safety_monitor_language_file;
    std::string active_recovery_plan;

    ros::Subscriber emergency_event_subscriber;
    ros::Subscriber recovery_plan_finished;    
    ros::ServiceClient stop_mission;
    ros::ServiceClient recovery_plan_client;
    ros::ServiceClient query_client;

    aerostack_msgs::StringStamped emergency_event_msg;

//Callbacks
    void emergencyEventCallback(const aerostack_msgs::StringStamped& msg);
    void behaviorActivationFinishedCallback(const behavior_execution_manager_msgs::BehaviorActivationFinished &message);
    void recoveryPlanFinishedCallback(const aerostack_msgs::RecoveryPlanFinished &message);
    std::vector<std::string> getsubs(std::vector<std::string> pairs);
    std::vector<std::string> getpairs(std::string subs);
    std::vector<std::string> getvars(std::vector<std::string> pairs);

};

#endif 



