/*!*******************************************************************************************
 *  \file       recovery_manager_process.cpp
 *  \brief      Recovery manager implementation file.
 *  \authors    Javier Cabrera Marugan
 *              Martín Molina
 *  \copyright  Copyright (c) 2019 Universidad Politecnica de Madrid
 *              All rights reserved
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

#include "recovery_manager_process.h"
RecoveryManagerProcess::RecoveryManagerProcess()
{}

RecoveryManagerProcess::~RecoveryManagerProcess()
{}

double RecoveryManagerProcess::get_moduleRate()
{ 
 return rate;
}

void RecoveryManagerProcess::ownSetUp()
{ 
  // Configs
  ros::param::get("~robot_namespace", robot_namespace);
  if ( robot_namespace.length() == 0) robot_namespace = "drone1";
  std::cout << "robot_namespace=" <<robot_namespace<< std::endl;
  ros::param::get("~frequency", rate);
  ros::param::get("~drone_id", id);
}

void RecoveryManagerProcess::ownStart(){    
  std::cout<<"STARTING RECOVERY MANAGER"<<std::endl;
  // Services  
  recovery_plan_server = n.advertiseService("/" + robot_namespace +"/execute_recovery_plan", &RecoveryManagerProcess::executeRecoveryPlan, this);
  // Clients of services
  activate_task_srv = n.serviceClient<behavior_coordinator_msgs::StartTask>("/" + robot_namespace +"/start_task");
  query_client = n.serviceClient<belief_manager_msgs::QueryBelief>("query_belief");
  remove_client = n.serviceClient<belief_manager_msgs::RemoveBelief>("remove_belief");
  recovery_plan_finished = n.advertise<aerostack_msgs::RecoveryPlanFinished>("/" + robot_namespace +"/recovery_plan_finished", 1000);
  current_plan="";
}

void RecoveryManagerProcess::ownStop(){}   

void RecoveryManagerProcess::ownRun(){}

void RecoveryManagerProcess::return_starting_point(){
  behavior_coordinator_msgs::StartTask::Request msg;
  behavior_coordinator_msgs::StartTask::Response res;
  behavior_coordinator_msgs::TaskCommand task;
  task.name = "HOVER";
  task.priority = 3;
  msg.task = task;
  activate_task_srv.call(msg,res);
  ros::Duration(1).sleep();
  task.name = "FOLLOW_PATH";
  task.priority = 3;
  task.parameters = "path: [[0, 0, 1]]";
  msg.task=task;
  activate_task_srv.call(msg,res);
  while(true){
    msg_from_activation = *ros::topic::waitForMessage<behavior_coordinator_msgs::TaskStopped>("/" + robot_namespace +"/task_stopped", n);
    if(msg_from_activation.name  == "FOLLOW_PATH") break;
  }
  task.name = "LAND";
  task.priority = 3;
  task.parameters = "";
  msg.task=task;
  activate_task_srv.call(msg,res);
  while(true){
    msg_from_activation = *ros::topic::waitForMessage<behavior_coordinator_msgs::TaskStopped>("/" + robot_namespace +"/task_stopped", n);
    if(msg_from_activation.name  == "LAND") break;
  }
  aerostack_msgs::RecoveryPlanFinished msg_finished;
  msg_finished.plan = "return_starting_point";
  recovery_plan_finished.publish(msg_finished);
  current_plan="";
}

void RecoveryManagerProcess::emergency_land(){
  behavior_coordinator_msgs::StartTask::Request msg;
  behavior_coordinator_msgs::StartTask::Response res;
  behavior_coordinator_msgs::TaskCommand task;
  task.name = "LAND";
  task.priority = 3;
  msg.task = task;
  activate_task_srv.call(msg,res);
  aerostack_msgs::RecoveryPlanFinished msg_finished;
  msg_finished.plan = "emergency_land";
  recovery_plan_finished.publish(msg_finished);
  current_plan="";
}

bool RecoveryManagerProcess::executeRecoveryPlan(aerostack_msgs::RecoveryPlan::Request& req,
                                        aerostack_msgs::RecoveryPlan::Response& res) {
  if(req.plan == "emergency_land"){
  	std::cout<<"execute emergency_land"<<std::endl;
  	emergency_land();
  }
  else if( req.plan == "return_starting_point")	{
  	std::cout<<"execute return_starting_point"<<std::endl;
    return_starting_point();
  }else{
  	return false;
  } 
  res.ack = true;
  res.plan_activated=req.plan;
  return true;
}


        