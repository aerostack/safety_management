/*!*******************************************************************************************
 *  \file       recovery_manager_process_main.cpp
 *  \brief      Safety monitor implementation file.
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

#include <iostream>
#include <math.h>

#include "ros/ros.h"
#include "recovery_manager_process.h"
#include "nodes_definition.h"



int main(int argc,char **argv)
{
    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle n;

    std::cout << "[ROSNODE] Starting "<<ros::this_node::getName() << std::endl;
       
    RecoveryManagerProcess recovery_manager;
    recovery_manager.setUp();
    
    recovery_manager.start();
    
    ros::Rate loop_rate(recovery_manager.get_moduleRate());
 
    try
    {    
        while(ros::ok())
        {  
            if(recovery_manager.getState()==STATE_RUNNING){
                recovery_manager.run();  
            } 
             ros::spinOnce();
             loop_rate.sleep(); 
        }
    }
    catch (std::exception &ex)
    {
        std::cout << "[ROSNODE] Exception :" << ex.what() << std::endl;
    }
    return 0;
}
