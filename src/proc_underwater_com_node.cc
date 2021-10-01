/**
 * \file	proc_underwater_com_node.cc
 * \author	Francis Alonzo <francisalonzo29@gmail.com
 * \date	02/09/2021
 * 
 * \copyright Copyright (c) 2021 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#include "proc_underwater_com_node.h"

namespace proc_underwater_com
{
    // Node Constructor
    ProcUnderwaterComNode::ProcUnderwaterComNode(const ros::NodeHandlePtr &_nh)
        : nh_(_nh)
    {
        underwaterComSubscriber_ = nh_->subscribe("/provider_underwater_com/receive_msgs", 100, &ProcUnderwaterComNode::UnderwaterComInterpreterCallback, this);
        stateKillSubcrisber_ = nh_->subscribe("/provider_kill_mission/kill_switch_msg", 100, &ProcUnderwaterComNode::StateKillCallback, this);
        stateMissionSubcrisber_ = nh_->subscribe("/provider_kill_mission/mission_switch_msg", 100, &ProcUnderwaterComNode::StateMissionCallback, this);
        depthSubcrisber_ = nh_->subscribe("/provider_depth/depth", 100, &ProcUnderwaterComNode::DepthCallback, this);

        underwaterComPublisher_ = nh_->advertise<std_msgs::String>("/proc_underwater_com/send_msgs", 100);
        auvStateKillPublisher_ = nh_->advertise<sonia_common::KillSwitchMsg>("/proc_underwater_com/other_auv_state_kill", 100);
        auvStateMissionPublisher_ = nh_->advertise<sonia_common::MissionSwitchMsg>("/proc_underwater_com/other_auv_state_mission", 100);
        auvDepthPublisher_ = nh_->advertise<std_msgs::Float32>("/proc_underwater_com/other_auv_depth", 100);

        underwaterComClient_ = nh_->serviceClient<sonia_common::ModemPacket>("/provider_underwater_com/request");

        underwaterComClient_.waitForExistence();
        
        sonia_common::ModemPacket srv;
        srv.request.cmd = CMD_GET_SETTINGS;

        if(GetSensorState(srv))
        {
            role_ = srv.response.role;
            ROS_INFO_STREAM("Role is setup");
        }
        else
        {
            ros::shutdown();
        }

        diagnostic_thread = std::thread(std::bind(&ProcUnderwaterComNode::Verify_Link, this));
    }

    // Node Destructor
    ProcUnderwaterComNode::~ProcUnderwaterComNode(){}

    // Spin
    void ProcUnderwaterComNode::Spin()
    {
        ros::Rate r(0.2); // 0.2 Hz or 5 sec between each send

        while(ros::ok())
        {
            if(role_ == ROLE_MASTER && link_ == LINK_UP && received_message_)
            {
                ROS_INFO_STREAM("Sending a message to the salve");
                SendMessage();
                received_mutex.lock();
                received_message_ = false;
                received_mutex.unlock();
            }
            ros::spinOnce();
            r.sleep();
        }
    }

    void ProcUnderwaterComNode::UnderwaterComInterpreterCallback(const std_msgs::String &msg)
    {
        bool auvStateKill = (std::stoi((msg.data.substr(5,1))) != 0);
        bool auvStateMission = (std::stoi((msg.data.substr(7,1))) != 0);
        float_t auvDepth = (std::stol(msg.data.substr(1,3))) / 100.0;

        AuvStateKillInterpreter(auvStateKill);
        AuvStateMissionInterpreter(auvStateMission);
        AuvDepthInterpreter(auvDepth);

        if(role_ == ROLE_SLAVE)
        {
            SendMessage();
        }
        else
        {
            received_mutex.lock();
            received_message_ = true;
            received_mutex.unlock();
        }
    }
    
    void ProcUnderwaterComNode::SendMessage()
    {
        std_msgs::String packet;
        uint16_t depth = (uint16_t)(lastDepth_ * 100.0);
        uint8_t kill = (lastStateKill_) ? 1 : 0;
        uint8_t mission = (lastStateMission_) ? 1 : 0;
        
        packet.data = "D" + std::to_string(depth) + "K" + std::to_string(kill) + "M" + std::to_string(mission);

        underwaterComPublisher_.publish(packet);
    }

    bool ProcUnderwaterComNode::GetSensorState(sonia_common::ModemPacket &srv)
    {
        if(underwaterComClient_.call(srv))
        {
            ROS_DEBUG_STREAM("Service called");
            return true;
        }
        else
        {
            ROS_ERROR_STREAM("Service can't be called");
            return false;
        }
    }

    void ProcUnderwaterComNode::AuvStateKillInterpreter(const bool state)
    {
        stateKill_.state = state;
        auvStateKillPublisher_.publish(stateKill_);
    }

    void ProcUnderwaterComNode::AuvStateMissionInterpreter(const bool state)
    {
        stateMission_.state = state;
        auvStateMissionPublisher_.publish(stateMission_);
    }

    void ProcUnderwaterComNode::AuvDepthInterpreter(const float_t data)
    {
        depth_.data = data;
        auvDepthPublisher_.publish(depth_);
    }

    void ProcUnderwaterComNode::StateKillCallback(const sonia_common::KillSwitchMsg &msg)
    {
        lastStateKill_ = msg.state;
    }

    void ProcUnderwaterComNode::StateMissionCallback(const sonia_common::MissionSwitchMsg &msg)
    {
        lastStateMission_ = msg.state;
    }

    void ProcUnderwaterComNode::DepthCallback(const std_msgs::Float32 &msg)
    {
        lastDepth_ = msg.data;
    }

    void ProcUnderwaterComNode::Verify_Link()
    {
        ros::Rate r(0.1); // 0.1 Hz
        sonia_common::ModemPacket srv, flush_srv;
        flush_srv.request.cmd = CMD_FLUSH;
        srv.request.cmd = CMD_GET_DIAGNOSTIC;

        while(!ros::isShuttingDown())
        {
            ROS_INFO_STREAM("Link is updated");
            if(GetSensorState(srv))
            {
                link_ = (char) srv.response.link;
            }
            if(link_ == LINK_DOWN)
            {
                ROS_INFO_STREAM("Link is down. Flushing queue"); 
                GetSensorState(flush_srv);
            }
            r.sleep();
        }
    }
}