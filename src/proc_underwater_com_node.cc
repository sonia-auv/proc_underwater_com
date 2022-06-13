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
        ioSubcrisber_ = nh_->subscribe("/provider_actuators/return_action", 100, &ProcUnderwaterComNode::IOCallback, this);
        
        // Advertisers
        underwaterComPublisher_ = nh_->advertise<std_msgs::UInt64>("/proc_underwater_com/send_msgs", 100);
        auvStateKillPublisher_ = nh_->advertise<std_msgs::Bool>("/proc_underwater_com/other_auv_state_kill", 100);
        auvStateMissionPublisher_ = nh_->advertise<std_msgs::Bool>("/proc_underwater_com/other_auv_state_mission", 100);
        auvDepthPublisher_ = nh_->advertise<std_msgs::Float32>("/proc_underwater_com/other_auv_depth", 100);
        auvIOPublisher_ = nh_->advertise<std_msgs::UInt8MultiArray>("/proc_underwater_com/other_auv_io", 100);

        // Service  
        underwaterComGetMissionList_ = nh_->advertiseService("/proc_underwater_com/get_mission_list", &ProcUnderwaterComNode::GetMissionList, this);
        underwaterComUpdateMissionList_ = nh_->advertiseService("/proc_underwater_com/update_mission_list", &ProcUnderwaterComNode::UpdateMissionList, this);
        underwaterComClient_ = nh_->serviceClient<sonia_common::ModemSendCmd>("/provider_underwater_com/request");

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

    void ProcUnderwaterComNode::UnderwaterComInterpreterCallback(const std_msgs::UInt64 &msg)
    {
        Modem_M64_t packet = ConstructPacket(msg.data);

        if(VerifyPacket(packet) >= 0)
        {
            bool auvStateKill = packet.killSwitchState;
            bool auvStateMission = packet.missionSwitchState;
            float_t auvDepth = (float_t)packet.depth / 100.0;
            UpdateMissionState(packet.missionId, packet.missionState);

            AuvStateKillInterpreter(auvStateKill);
            AuvStateMissionInterpreter(auvStateMission);
            AuvDepthInterpreter(auvDepth);
        }
    }
    
    void ProcUnderwaterComNode::SendMessage()
    {
        Modem_M64_t send_packet;
        std_msgs::UInt64 msg;

        send_packet.header.packetNumber = 0b1;
        send_packet.header.packetId = 0b1;
        send_packet.header.endOfPacket = 0b1;
        send_packet.depth = (uint16_t)(lastDepth_ * 100.0);
        send_packet.killSwitchState = lastStateKill_;
        send_packet.missionSwitchState = lastStateMission_;
        send_packet.missionId = SendMissionState();
        send_packet.missionState = mission_state.at(send_packet.missionId);
        send_packet.droppersState = (lastIO_ & 0x0F);
        send_packet.torpedosState = (lastIO_ & 0xF0);

        msg.data = DeconstructPacket(send_packet);
        underwaterComPublisher_.publish(msg);
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

    void ProcUnderwaterComNode::AuvIOInterpreter(const uint8_t data)
    {
        std_msgs::UInt8MultiArray msg;

        msg.data.clear();
        msg.layout.dim[0].label = io_activation;

        for(uint8_t i = 0; i < 4; ++i)
        {
            msg.data.push_back((data >> (i*2)) & 0x01);
        }

        auvIOPublisher_.publish(msg);
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

    void ProcUnderwaterComNode::IOCallback(const sonia_common::ActuatorDoAction & msg) // Not tested as no implementation existed
    {
        if(msg.element == sonia_common::ActuatorDoAction::ELEMENT_DROPPER)
        {
            if(msg.side == sonia_common::ActuatorDoAction::SIDE_STARBOARD) lastIO_ = lastIO_ | msg.action;
            if(msg.side == sonia_common::ActuatorDoAction::SIDE_PORT) lastIO_ = lastIO_ | (msg.action << 2);
        }
        else if(msg.element == sonia_common::ActuatorDoAction::ELEMENT_TORPEDO)
        {
            if(msg.side == sonia_common::ActuatorDoAction::SIDE_STARBOARD) lastIO_ = lastIO_ | (msg.action << 4);
            if(msg.side == sonia_common::ActuatorDoAction::SIDE_PORT) lastIO_ = lastIO_ | (msg.action << 6);
        }
    }

    void ProcUnderwaterComNode::Process()
    {
        ros::Rate r(0.2); // 0.2 Hz or 5 sec between each send
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
    
    Modem_M64_t ProcUnderwaterComNode::ConstructPacket(const uint64_t data)
    {
        return *((Modem_M64_t *)&data);
    }
    
    uint64_t ProcUnderwaterComNode::DeconstructPacket(const Modem_M64_t packet)
    {
        return *((uint64_t *)&packet);
    }

    int8_t ProcUnderwaterComNode::VerifyPacket(const Modem_M64_t packet)
    {
        uint8_t nbPacket = packet.header.packetNumber;
        uint8_t packetId = packet.header.packetId;

        // Verification of the packet is conform to the protocol
        if(nbPacket < 1 || packetId < 1)
        {
            ROS_WARN_STREAM("Problem with the packet received. Dropping packet");
            return -EINVAL;
        }
        return packetId;
    }

    void ProcUnderwaterComNode::InitMissionState(uint8_t size)
    {
        mission_state.resize(size);
        size_mission_state = size;
    }

    uint8_t ProcUnderwaterComNode::SendMissionState()
    {
        return (index_++) % size_mission_state;
    }

    void ProcUnderwaterComNode::UpdateMissionState(uint8_t index, int8_t state)
    {
        mission_state.at(index) = state;
    }
}