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
        depthSubcrisber_ = nh_->subscribe("/provider_depth/depth", 100, &ProcUnderwaterComNode::DepthCallback, this);
        updateMissionSubcrisber_ = nh_->subscribe("/proc_underwater_com/mission_state_msg", 100, &ProcUnderwaterComNode::MissionStateCallback, this);
        syncSubscriber_ = nh_->subscribe("/proc_underwater_com/send_sync_request", 100, &ProcUnderwaterComNode::SyncCallback, this);
        
        // Advertisers
        underwaterComPublisher_ = nh_->advertise<std_msgs::UInt64>("/proc_underwater_com/send_msgs", 100);
        auvMissionPublisher_ = nh_->advertise<std_msgs::Int8MultiArray>("/proc_underwater_com/sub_mission_list", 100, true);
        otherauvMissionPublisher_ = nh_->advertise<std_msgs::Int8MultiArray>("/proc_underwater_com/other_sub_mission_list", 100, true);
        syncPublisher_ =  nh_->advertise<std_msgs::Bool>("/proc_underwater_com/sync_requested", 100, true);
        DepthPublisher_ =  nh_->advertise<std_msgs::Float32>("/proc_underwater_com/other_sub_depth", 100);

        // Service
        depthSrv_ = nh_->advertiseService("/proc_underwater_com/depth_request", &ProcUnderwaterComNode::DepthRequest, this);
        underwaterComClient_ = nh_->serviceClient<sonia_common::ModemSendCmd>("/provider_underwater_com/request");

        InitMissionState(configuration_.getNumberMission());
        AUVID = configuration_.getNumberid();
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
        float_t auvDepth = 0;
        uint32_t temp = 0;
        uint8_t data[] = {packet.data[0], packet.data[1], packet.data[2], packet.data[3]};

        if(VerifyPacket(packet))
        {
            switch (packet.cmd){

                case mission: 
                    UpdateMissionState_othersub(packet.data[0], packet.data[1]);
                break; 

                case depth:
                    //receive depth from other sub and publish it
                    if(packet.rec_send == 0){  
                        memcpy(&temp,&data, sizeof(uint32_t));
                        auvDepth = (float_t)temp / 100.0;
                        AuvDepthInterpreter(auvDepth);

                    //request depth from other sub
                    } else if (packet.rec_send == 1){
                        SendDepth();
                    }
                break;
                case sync: 
                    AuvSyncInterpreter(packet.rec_send);
                break;

                default:
                    ROS_INFO("Unknown command received: No action associated");
                break;
                 
        }  
            
        }
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


    void ProcUnderwaterComNode::AuvDepthInterpreter(const float_t data)
    {
        std_msgs::Float32 Depth;
        other_sub_depth_ = data;
        Depth.data = other_sub_depth_;
        DepthPublisher_.publish(Depth);
    }

    void ProcUnderwaterComNode::DepthCallback(const std_msgs::Float32 &msg)
    {
        lastDepth_ = msg.data;
    }

    
    Modem_M64_t ProcUnderwaterComNode::ConstructPacket(const uint64_t data)
    {
        return *((Modem_M64_t *)&data);
    }
    
    uint64_t ProcUnderwaterComNode::DeconstructPacket(const Modem_M64_t packet)
    {
        return *((uint64_t *)&packet);
    }


    void ProcUnderwaterComNode::InitMissionState(uint8_t size)
    {
        mission_state.data.resize(size); 
        other_sub_mission_state.data.resize(size);
        size_mission_state = size;
    }


    void ProcUnderwaterComNode::UpdateMissionState(uint8_t index, int8_t state)
    {
        mission_state.data[index] = state;
        auvMissionPublisher_.publish(mission_state); 
    }

    void ProcUnderwaterComNode::UpdateMissionState_othersub(uint8_t index, int8_t state)
    {
        other_sub_mission_state.data[index] = state;
        otherauvMissionPublisher_.publish(other_sub_mission_state); 
    }

    int8_t ProcUnderwaterComNode::VerifyPacket(const Modem_M64_t packet){
        
        if(packet.AUV_ID == 7 || packet.AUV_ID == 8 ){
            return 1;
        } 
        else{
            return 0;
        }
    }

    void ProcUnderwaterComNode::AuvSyncInterpreter(const bool state){
        std_msgs::Bool sync_status;
        sync_status.data = state;
        syncPublisher_.publish(sync_status);
    }

    void ProcUnderwaterComNode::MissionStateCallback(const sonia_common::ModemUpdateMissionList &msg){ 
         
        Modem_M64_t send_packet;
        std_msgs::UInt64 send_msg;

        send_packet.AUV_ID = AUVID; 
        send_packet.cmd = mission;
        send_packet.data[0] = msg.mission_id;
        send_packet.data[1] = msg.mission_state;

        send_msg.data = DeconstructPacket(send_packet);
        underwaterComPublisher_.publish(send_msg);
        UpdateMissionState(msg.mission_id,msg.mission_state);
     }

    void ProcUnderwaterComNode::SyncCallback(const std_msgs::Bool &msg){
         
         if (msg.data == true){
            Modem_M64_t send_packet;
            std_msgs::UInt64 send_msg;

            send_packet.AUV_ID = AUVID; 
            send_packet.cmd = sync;
            send_packet.rec_send = 1;

            send_msg.data = DeconstructPacket(send_packet);
            underwaterComPublisher_.publish(send_msg);
         }
     }

     bool ProcUnderwaterComNode::DepthRequest(std_srvs::Empty::Request &DepthRsq, std_srvs::Empty::Response &DepthRsp){
        
        Modem_M64_t send_packet;
        std_msgs::UInt64 send_msg;

        send_packet.AUV_ID = AUVID; 
        send_packet.cmd = depth;
        send_packet.rec_send = 1;

        send_msg.data = DeconstructPacket(send_packet);
        underwaterComPublisher_.publish(send_msg);

        return true;
     }

     void ProcUnderwaterComNode::SendDepth(){

        Modem_M64_t send_packet;
        std_msgs::UInt64 send_msg;
        uint8_t data[4] = {0,0,0,0};
        uint32_t auvDepth;
        auvDepth = lastDepth_ * 100.0; 

        memcpy(&data,&auvDepth, sizeof(uint32_t));
        
        send_packet.AUV_ID = AUVID; 
        send_packet.cmd = depth;
        send_packet.rec_send = 0;
        send_packet.data[0]=data[0];
        send_packet.data[1]=data[1];
        send_packet.data[2]=data[2];
        send_packet.data[3]=data[3];

        send_msg.data = DeconstructPacket(send_packet);
        underwaterComPublisher_.publish(send_msg);
     }

}