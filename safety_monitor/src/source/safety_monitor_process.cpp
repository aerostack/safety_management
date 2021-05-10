/*!*******************************************************************************************
 *  \file       emergency_manager_process.cpp
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

#include "safety_monitor_process.h"

SafetyMonitorProcess::SafetyMonitorProcess()
{}

SafetyMonitorProcess::~SafetyMonitorProcess()
{}

double SafetyMonitorProcess::get_moduleRate()
{ 
 return rate;
}

void SafetyMonitorProcess::ownSetUp()
{   
  // Configs
  ros::param::get("~robot_namespace", robot_namespace);
  if ( robot_namespace.length() == 0) robot_namespace = "drone1";
  std::cout << "robot_namespace=" <<robot_namespace<< std::endl;
  ros::param::get("~safety_monitor_language_path",safety_monitor_language_path);
  ros::param::get("~safety_monitor_language_file",safety_monitor_language_file);
  ros::param::get("~frequency", rate);
}

void SafetyMonitorProcess::ownStart(){    
  std::cout<<"STARTING SAFETY MONITOR"<<std::endl;
  
  emergency_event_subscriber = n.subscribe("/"+robot_namespace+"/emergency_event", 1, &SafetyMonitorProcess::emergencyEventCallback, this);
  stop_mission = n.serviceClient<std_srvs::Empty>("/"+robot_namespace+"/python_based_mission_interpreter_process/stop");
  recovery_plan_client = n.serviceClient<aerostack_msgs::RecoveryPlan>("/" + robot_namespace +"/execute_recovery_plan");
  recovery_plan_finished = n.subscribe("/"+robot_namespace+"/recovery_plan_finished", 1, &SafetyMonitorProcess::recoveryPlanFinishedCallback, this);
  query_client = n.serviceClient<belief_manager_msgs::QueryBelief>("query_belief");
  active_recovery_plan ="";
  try{
    safety_rules = YAML::LoadFile(safety_monitor_language_path+"/"+safety_monitor_language_file);
  }catch(std::exception &e){
    std::cout<<"Failed to load yaml file: "<<safety_monitor_language_file<<std::endl;
    return;
  }

}

void SafetyMonitorProcess::ownStop(){}   

void SafetyMonitorProcess::ownRun(){}


void SafetyMonitorProcess::recoveryPlanFinishedCallback(const aerostack_msgs::RecoveryPlanFinished &message){
  active_recovery_plan="";
}
std::vector<std::string> SafetyMonitorProcess::getpairs(std::string subs){
std::vector<std::string> recortes;
   int ini=0;
   int pos=0;
    while((pos=subs.find("\n",pos))!=std::string::npos){
      recortes.push_back(subs.substr(ini,pos-ini));
      pos=pos+1;
      ini=pos;

  }
//now we are going to delete spaces
  std::vector<std::string> res;
  for(int j=0;j<recortes.size();j++){
      std::string aux="";
      for(int  i = 0; recortes[j][i] != 0;i++){
              if(recortes[j][i] != 32){
                  aux=aux+recortes[j][i];
              }
      }
      res.push_back(aux);
  }

return res;
}

std::vector<std::string> SafetyMonitorProcess::getvars(std::vector<std::string> pairs){

  std::vector<std::string>res;
  for (int i=0; i<pairs.size();i++){
    res.push_back(pairs[i].substr(0,pairs[i].find(":")));
  }

   for (int i=0; i<pairs.size();i++){
       res[i]="?"+res[i];
   }

  return res;
}
std::vector<std::string> SafetyMonitorProcess::getsubs(std::vector<std::string> pairs){

  std::vector<std::string>res;
  for (int i=0; i<pairs.size();i++){
     res.push_back(pairs[i].substr(pairs[i].find(":")+1,pairs[i].size()-1));
  }
  return res;
}
//Callbacks
void SafetyMonitorProcess::emergencyEventCallback(const aerostack_msgs::StringStamped& msg){
    //Get msg
    std::string belief_predicate = ""; 
    std::string belief_value = ""; 
    bool pred_bool = true;
    bool pred_value = false;
    for (auto x : msg.data){ 
        if (x == '(') pred_bool = false;
        if (x == ')') pred_value = false;
        if (pred_bool) belief_predicate = belief_predicate + x; 
        if (pred_value && x != ' ') belief_value = belief_value + x; 
        if (x == ',') pred_value = true;   
    }
    YAML::Node recovery_actions;
    YAML::Node activations;
    YAML::Node parameters;
    YAML::Node item_parameters;
    std::map<std::string,std::string> map_parameters;
    if(!(recovery_actions = safety_rules["recovery_actions_for_emergency_events"])){
        std::cout<<"error, no recovery_actions_for_emergency_events";
    }
    for (std::size_t i=0;i<recovery_actions.size();i++){
        YAML::Node recovery_action = recovery_actions[i];
        if((recovery_action["predicate"].as<std::string>()==belief_predicate) && (!recovery_action["value"] || recovery_action["value"].as<std::string>()==belief_value)) {
          if(recovery_action["abort_mission"].as<std::string>() == "TRUE"){
    			 //Stop mission
    			  std_srvs::Empty stop_mission_msg;
    			  stop_mission.call(stop_mission_msg);
          }
          if(recovery_action["recovery_plan"]){
            aerostack_msgs::RecoveryPlan::Request msg_rec;
            aerostack_msgs::RecoveryPlan::Response res_rec;
            std::string plan= recovery_action["recovery_plan"].as<std::string>();
            msg_rec.plan = plan;
            recovery_plan_client.call(msg_rec,res_rec);
            if(!res_rec.ack){
              std::cout<<"[SAFETY MONITOR] Recovery plan not found"<<std::endl;
                return;
            }
            active_recovery_plan=res_rec.plan_activated; // store the current recovery plan when it is correctly activated
          }
        }
    }
}
        