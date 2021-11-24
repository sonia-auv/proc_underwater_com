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
        : nh_(_nh), 
          configuration_(_nh)
    {
        // Subscribers
        underwaterComSubscriber_ = nh_->subscribe("/provider_underwater_com/receive_msgs", 100, &ProcUnderwaterComNode::UnderwaterComInterpreterCallback, this);
        stateKillSubcrisber_ = nh_->subscribe("/provider_kill_mission/kill_switch_msg", 100, &ProcUnderwaterComNode::StateKillCallback, this);
        stateMissionSubcrisber_ = nh_->subscribe("/provider_kill_mission/mission_switch_msg", 100, &ProcUnderwaterComNode::StateMissionCallback, this);
        depthSubcrisber_ = nh_->subscribe("/provider_depth/depth", 100, &ProcUnderwaterComNode::DepthCallback, this);
        
        // Advertisers
        underwaterComPublisher_ = nh_->advertise<std_msgs::String>("/proc_underwater_com/send_msgs", 100);
        auvStateKillPublisher_ = nh_->advertise<sonia_common::KillSwitchMsg>("/proc_underwater_com/other_auv_state_kill", 100);
        auvStateMissionPublisher_ = nh_->advertise<sonia_common::MissionSwitchMsg>("/proc_underwater_com/other_auv_state_mission", 100);
        auvDepthPublisher_ = nh_->advertise<std_msgs::Float32>("/proc_underwater_com/other_auv_depth", 100);

        // Service  
        underwaterComGetMissionList_ = nh_->advertiseService("/proc_underwater_com/get_mission_list", &ProcUnderwaterComNode::GetMissionList, this);
        underwaterComUpdateMissionList_ = nh_->advertiseService("/proc_underwater_com/update_mission_list", &ProcUnderwaterComNode::UpdateMissionList, this);
        underwaterComClient_ = nh_->serviceClient<sonia_common::ModemSendCmd>("/provider_underwater_com/request");
        underwaterComClient_.waitForExistence();

        ros::Duration(10).sleep(); // Wait for default config to be done

        ROS_INFO_STREAM("Settings up the role for the sensor");
        
        sonia_common::ModemSendCmd srv;
        srv.request.cmd = CMD_SET_SETTINGS;
        srv.request.role = (uint8_t) configuration_.getRole().at(0);
        srv.request.channel = std::stoi(configuration_.getChannel());

        if(SensorState(srv))
        {
            role_ = srv.response.role;
            ROS_INFO("Role is : %c", role_);
            ROS_INFO("Channel is : %d", srv.response.channel);
        }
        else
        {
            ros::shutdown();
        }

        process_thread = std::thread(std::bind(&ProcUnderwaterComNode::Process, this));
    }

    // Node Destructor
    ProcUnderwaterComNode::~ProcUnderwaterComNode(){}

    // Spin
    void ProcUnderwaterComNode::Spin()
    {
        ros::Rate r(5); // 5 Hz

        while(ros::ok())
        {
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
    }
    
    void ProcUnderwaterComNode::SendMessage()
    {
        std_msgs::String packet;
        std::stringstream ss;
        uint16_t depth = (uint16_t)(lastDepth_ * 100.0);
        char depth_buffer[3];
        uint8_t kill = (lastStateKill_) ? 1 : 0;
        uint8_t mission = (lastStateMission_) ? 1 : 0;

        sprintf(depth_buffer, "%03u", depth);
        ss << "D" << depth_buffer << "K" << std::to_string(kill) << "M" << std::to_string(mission);
        packet.data = ss.str();

        underwaterComPublisher_.publish(packet);
    }

    bool ProcUnderwaterComNode::SensorState(sonia_common::ModemSendCmd &srv)
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

    bool ProcUnderwaterComNode::GetMissionList(sonia_common::ModemGetMissionList::Request &req, sonia_common::ModemGetMissionList::Response &res)
    {
        copy(mission_state.begin(), mission_state.end(), back_inserter(res.state));
        return true;
    }

    bool ProcUnderwaterComNode::UpdateMissionList(sonia_common::ModemUpdateMissionList::Request &req, sonia_common::ModemUpdateMissionList::Response &res)
    {
        mission_state.at(req.mission_id) = req.mission_state;
        res.array_updated = true; // For future use
        return true;
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

    void ProcUnderwaterComNode::Process()
    {
        ros::Rate r(0.2); // 0.2 Hz or 5 secondes entre chaque envoi
        sonia_common::ModemSendCmd srv;
        srv.request.cmd = CMD_GET_DIAGNOSTIC;

        while(!ros::isShuttingDown())
        {
            if(SensorState(srv))
            {
                ROS_INFO_STREAM("Verifying link status");
                link_ = (char) srv.response.link;
            }
            if(link_ == LINK_UP)
            {
                ROS_INFO_STREAM("Sending message");
                SendMessage();
            }
            r.sleep();
        }
    }
}