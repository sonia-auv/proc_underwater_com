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
        underwaterComPublisher_ = nh_->advertise<sonia_common::IntersubCom>("/proc_underwater_com/send_msgs", 100);
        auvStateKillPublisher_ = nh_->advertise<std_msgs::Bool>("/proc_underwater_com/other_auv_state_kill", 100);
        auvStateMissionPublisher_ = nh_->advertise<std_msgs::Bool>("/proc_underwater_com/other_auv_state_mission", 100);
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
            ROS_INFO("Role is : %c", srv.response.role);
            ROS_INFO("Channel is : %d", srv.response.channel);
        }
        else
        {
            ros::shutdown();
        }

        InitMissionState(configuration_.getNumberMission());
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

    void ProcUnderwaterComNode::UnderwaterComInterpreterCallback(const sonia_common::IntersubCom &msg)
    {
        bool auvStateKill = msg.kill_switch_state;
        bool auvStateMission = msg.mission_switch_state;
        float_t auvDepth = msg.depth * 100.0;
        UpdateMissionState(msg.mission_id, msg.mission_state);

        AuvStateKillInterpreter(auvStateKill);
        AuvStateMissionInterpreter(auvStateMission);
        AuvDepthInterpreter(auvDepth);
    }
    
    void ProcUnderwaterComNode::SendMessage()
    {
        intercom_msg_.depth = (uint16_t)(lastDepth_ * 100.0);
        intercom_msg_.kill_switch_state = lastStateKill_;
        intercom_msg_.mission_switch_state = lastStateMission_;
        intercom_msg_.mission_id = SendMissionState();
        intercom_msg_.mission_state = mission_state.at(intercom_msg_.mission_id);

        underwaterComPublisher_.publish(intercom_msg_);
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
        stateKill_.data = state;
        auvStateKillPublisher_.publish(stateKill_);
    }

    void ProcUnderwaterComNode::AuvStateMissionInterpreter(const bool state)
    {
        stateMission_.data = state;
        auvStateMissionPublisher_.publish(stateMission_);
    }

    void ProcUnderwaterComNode::AuvDepthInterpreter(const float_t data)
    {
        depth_.data = data;
        auvDepthPublisher_.publish(depth_);
    }

    void ProcUnderwaterComNode::StateKillCallback(const std_msgs::Bool &msg)
    {
        lastStateKill_ = msg.data;
    }

    void ProcUnderwaterComNode::StateMissionCallback(const std_msgs::Bool &msg)
    {
        lastStateMission_ = msg.data;
    }

    void ProcUnderwaterComNode::DepthCallback(const std_msgs::Float32 &msg)
    {
        lastDepth_ = msg.data;
    }

    void ProcUnderwaterComNode::Process()
    {
        ros::Rate r(0.2); // 0.2 Hz or 5 secondes entre chaque envoi
        char link;
        sonia_common::ModemSendCmd srv;
        srv.request.cmd = CMD_GET_DIAGNOSTIC;

        while(!ros::isShuttingDown())
        {
            if(SensorState(srv))
            {
                ROS_INFO_STREAM("Verifying link status");
                link = (char) srv.response.link;
            }
            if(link == LINK_UP)
            {
                ROS_INFO_STREAM("Sending message");
                SendMessage();
            }
            r.sleep();
        }
    }

    void ProcUnderwaterComNode::InitMissionState(uint8_t size)
    {
        mission_state.resize(size);
        size_mission_state = size;
    }

    uint8_t ProcUnderwaterComNode::SendMissionState()
    {
        return (index_ + 1) % size_mission_state;
    }

    void ProcUnderwaterComNode::UpdateMissionState(uint8_t index, int8_t state)
    {
        mission_state.at(index) = state;
    }
}