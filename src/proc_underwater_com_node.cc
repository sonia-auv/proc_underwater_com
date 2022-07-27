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
        MissionInitSubscriber_ = nh_->subscribe("/proc_underwater_com/mission_init", 100, &ProcUnderwaterComNode::MissionInitCallback, this);
        
        // Advertisers
        underwaterComPublisher_ = nh_->advertise<std_msgs::UInt64>("/proc_underwater_com/send_msgs", 100);
        auvMissionPublisher_ = nh_->advertise<std_msgs::Int8MultiArray>("/proc_underwater_com/sub_mission_list", 100, true);
        otherauvMissionPublisher_ = nh_->advertise<std_msgs::Int8MultiArray>("/proc_underwater_com/other_sub_mission_list", 100, true);
        syncPublisher_ =  nh_->advertise<std_msgs::Bool>("/proc_underwater_com/sync_requested", 100);
        DepthPublisher_ =  nh_->advertise<std_msgs::Float32>("/proc_underwater_com/other_sub_depth", 100);

        // Services
        depthSrv_ = nh_->advertiseService("/proc_underwater_com/depth_request", &ProcUnderwaterComNode::DepthRequest, this);
        underwaterComClient_ = nh_->serviceClient<sonia_common::ModemSendCmd>("/provider_underwater_com/request");

        // Threads
        interpretMessage_ = std::thread(std::bind(&ProcUnderwaterComNode::UnderwaterComInterpreter, this));
        sendMessageToSensor_ = std::thread(std::bind(&ProcUnderwaterComNode::SendMessageToSensor, this));

        InitMissionState(configuration_.getNumberMission());
        AUVID = configuration_.getNumberid();

        ackknowledge_mission_completed_ = 0;
        ackknowledge_sync_completed_ = 0;
    }

    // Node Destructor
    ProcUnderwaterComNode::~ProcUnderwaterComNode()
    {
        stop_thread = true;
    }

    // Spin
    void ProcUnderwaterComNode::Spin()
    {
        ros::Rate r(20); // 20 Hz

        while(ros::ok())
        {
            ros::spinOnce();
            r.sleep();
        }
    }
    void ProcUnderwaterComNode::UnderwaterComInterpreterCallback(const std_msgs::UInt64 &msg)
    {
        ROS_DEBUG_STREAM("Received message");
        queue.push_back(msg.data);
    }

    void ProcUnderwaterComNode::UnderwaterComInterpreter()
    {
        Modem_M64_t packet;
        float_t auvDepth = 0;
        uint32_t temp = 0;
        uint8_t read_write = 0;
        uint8_t data[] = {0,0,0,0,0,0};

        while(!stop_thread)
        {
            if(!queue.empty())
            {
                packet = ConstructPacket(queue.get_n_pop_front());
                auvDepth = 0;
                temp = 0;
                read_write = packet.rec_send;
                
                for(uint8_t i = 0; i < 4; ++i)
                {
                    data[i] = packet.data[i];
                }

                if(VerifyPacket(packet))
                {
                    if(read_write == 1)
                    {
                        switch (packet.cmd)
                        {
                            case mission: 
                                UpdateMissionState_othersub(packet.data[0], packet.data[1]);
                                SendAckknowledge(packet.cmd);
                                break;
                            case depth:
                                //request depth from other sub
                                SendDepth();
                                break;
                            case sync: 
                                AuvSyncInterpreter(packet.rec_send);
                                SendAckknowledge(packet.cmd);
                                break;
                            default:
                                ROS_WARN_STREAM("Unknown command received: No action associated");
                                break;
                        }
                    }
                    else if(read_write == 0)
                    {
                        switch (packet.cmd)
                        {
                            case mission:
                                UpdateMissionState(packet.data[0], packet.data[1]);
                                SendAckknowledge(packet.cmd);
                                break;
                            case depth:
                                //receive depth from other sub and publish it
                                memcpy(&temp,&data, sizeof(uint32_t));
                                auvDepth = (float_t)temp / 100.0;
                                AuvDepthInterpreter(auvDepth);
                                break;
                            case sync:
                                break;
                            case ack:
                                if(data[0] == mission)
                                {
                                    ackknowledge_mission_completed_ = 0;
                                }
                                else if(data[0] == sync)
                                {
                                    ackknowledge_sync_completed_ = 0;
                                }
                                else
                                {
                                    ROS_DEBUG_STREAM("No acknowledge required for this command.");
                                }
                                break;
                            default:
                                ROS_WARN_STREAM("Unknown command received: No action associated");
                                break;
                        }
                    }
                    else
                    {
                        ROS_ERROR_STREAM("Error with the packet. Dropping packet");
                    }
                }
            }
            else
            {
                ros::Duration(0.1).sleep(); //ROSPARAM TO DO
            }
        }
    }

    void ProcUnderwaterComNode::SendMessageToSensor()
    {
        Modem_M64_t packet;
        std_msgs::UInt64 msg;
        std::chrono::seconds wait_period(configuration_.getDelayAck());

        while(!stop_thread)
        {
            if(!sendQueue_.empty())
            {
                packet = sendQueue_.get_n_pop_front();
                if(packet.cmd == mission)
                {
                    ackknowledge_mission_completed_++;
                    msg.data = DeconstructPacket(packet);
                    ROS_DEBUG_STREAM("Mutex acquired for mission and waiting on feedback");
                    while(ackknowledge_mission_completed_ != 0)
                    {
                        underwaterComPublisher_.publish(msg);
                        std::this_thread::sleep_for(wait_period);
                    }
                    ROS_DEBUG_STREAM("Mutex release for mission");
                }
                else if(packet.cmd == sync)
                {
                    ackknowledge_sync_completed_++;
                    msg.data = DeconstructPacket(packet);
                    ROS_DEBUG_STREAM("Mutex acquired for sync and waiting on feedback");
                    while(ackknowledge_sync_completed_ != 0)
                    {
                        underwaterComPublisher_.publish(msg);
                        std::this_thread::sleep_for(wait_period);
                    }
                    ROS_DEBUG_STREAM("Mutex release for sync");
                }
                else
                {
                    ROS_DEBUG_STREAM("Cmd doesn't require a acknowledge.");
                    msg.data = DeconstructPacket(packet);
                    underwaterComPublisher_.publish(msg);
                }
            }
            else
            {
                ros::Duration(0.1).sleep(); //ROSPARAM TO DO
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

    void ProcUnderwaterComNode::MissionInitCallback(const std_msgs::Int8MultiArray &msg)
    {
        if(uint16_t(msg.data.size()) == uint16_t(configuration_.getNumberMission()))
        {
            if(stoi(msg.layout.dim[0].label) == AUVID)
            {
                ROS_INFO_STREAM("Submarine mission list updated.");
                mission_state.data = msg.data;
                auvMissionPublisher_.publish(mission_state);

                int arraysize = sizeof(mission_state.data)/sizeof(mission_state.data[0]);

                //Send all mission states of this sub to the other sub
                for (uint8_t i = 0; i < arraysize; i++){

                    Modem_M64_t send_packet;

                    send_packet.AUV_ID = AUVID; 
                    send_packet.cmd = mission;
                    send_packet.rec_send =1;
                    send_packet.data[0] = i;
                    send_packet.data[1] = mission_state.data[i];
                    sendQueue_.push_back(send_packet);
                }
            }
            else
            {
                ROS_INFO_STREAM("Other submarine mission list updated.");
                other_sub_mission_state.data = msg.data;
                otherauvMissionPublisher_.publish(other_sub_mission_state);     

                int arraysize = sizeof(other_sub_mission_state.data)/sizeof(other_sub_mission_state.data[0]);

                //Send all mission states to other sub
                for (uint8_t i = 0; i < arraysize; i++){

                    Modem_M64_t send_packet;

                    send_packet.AUV_ID = AUVID; 
                    send_packet.cmd = mission;
                    send_packet.rec_send =0;
                    send_packet.data[0] = i;
                    send_packet.data[1] = other_sub_mission_state.data[i];
                    sendQueue_.push_back(send_packet);
                }           
            }
        }
        else
        {
            ROS_ERROR_STREAM("Number of tasks doesn't match the proc. Only " << std::to_string(msg.data.size()) << " tasks. Dropping message!");
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


    void ProcUnderwaterComNode::InitMissionState(uint8_t size)
    {
        mission_state.data.resize(size); 
        other_sub_mission_state.data.resize(size);
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

    int8_t ProcUnderwaterComNode::VerifyPacket(const Modem_M64_t packet)
    {    
        if(packet.AUV_ID == 7 || packet.AUV_ID == 8 )
        {
            return 1;
        } 
        else
        {
            return 0;
        }
    }

    void ProcUnderwaterComNode::AuvSyncInterpreter(const bool state)
    {
        std_msgs::Bool sync_status;
        sync_status.data = state;
        syncPublisher_.publish(sync_status);
    }

    void ProcUnderwaterComNode::MissionStateCallback(const sonia_common::ModemUpdateMissionList &msg)
    {      
        Modem_M64_t send_packet;

        send_packet.AUV_ID = AUVID; 
        send_packet.cmd = mission;
        send_packet.rec_send =1;
        send_packet.data[0] = msg.mission_id;
        send_packet.data[1] = msg.mission_state;

        UpdateMissionState(msg.mission_id,msg.mission_state);
        sendQueue_.push_back(send_packet);
     }

    void ProcUnderwaterComNode::SyncCallback(const std_msgs::Bool &msg)
    {     
        if (msg.data == true)
        {
            Modem_M64_t send_packet;

            send_packet.AUV_ID = AUVID; 
            send_packet.cmd = sync;
            send_packet.rec_send = 1;

            sendQueue_.push_back(send_packet);
        }
    }

    bool ProcUnderwaterComNode::DepthRequest(std_srvs::Empty::Request &DepthRsq, std_srvs::Empty::Response &DepthRsp)
    {    
        Modem_M64_t send_packet;

        send_packet.AUV_ID = AUVID; 
        send_packet.cmd = depth;
        send_packet.rec_send = 1;

        sendQueue_.push_back(send_packet);
        return true;
    }

    void ProcUnderwaterComNode::SendDepth()
    {
        Modem_M64_t send_packet;
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

        sendQueue_.push_back(send_packet);
    }

    void ProcUnderwaterComNode::SendAckknowledge(uint8_t cmd)
    {
        Modem_M64_t send_packet;
        std_msgs::UInt64 send_msg;

        send_packet.AUV_ID = AUVID;
        send_packet.cmd = ack;
        send_packet.rec_send = 0;
        send_packet.data[0] = cmd;
        send_packet.data[1]=0;
        send_packet.data[2]=0;
        send_packet.data[3]=0;

        send_msg.data = DeconstructPacket(send_packet);
        // Here we bypass the send thread to make sure that we don't get a deadlock
        underwaterComPublisher_.publish(send_msg);
    }

    void ProcUnderwaterComNode::ConfirmPacketReceived(uint8_t cmd)
    {
        if(cmd == mission)
        {
            ROS_DEBUG_STREAM("Received acknowledge for mission.");
            ackknowledge_mission_completed_--;
        }
        else if(cmd == sync)
        {
            ackknowledge_sync_completed_--;
        }
        else
        {
            ROS_DEBUG_STREAM("No acknowledge required for this command.");
        }
    }
}
