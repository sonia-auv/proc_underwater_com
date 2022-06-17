/**
 * \file	proc_underwater_com_node.h
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

#ifndef PROC_UNDERWATER_COM_NODE
#define PROC_UNDERWATER_COM_NODE

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_srvs/Empty.h>
#include <string>
#include <thread>
#include <mutex>
#include <algorithm>
#include <iterator>
#include <errno.h>

#include "Configuration.h"
#include "modem_data.h"
#include <sonia_common/Modem_Definitions.h>
// #include <sonia_common/ActuatorDoAction.h>
#include <sonia_common/ModemSendCmd.h>
//#include <sonia_common/ModemGetMissionList.h>
#include <sonia_common/ModemUpdateMissionList.h>

#define SIZE_UINT8 256

namespace proc_underwater_com {

enum command {mission,depth, sync};

class ProcUnderwaterComNode
{
    public:

        ProcUnderwaterComNode(const ros::NodeHandlePtr & _nh);
        ~ProcUnderwaterComNode();

        void Spin();

    private:

        void UnderwaterComInterpreterCallback(const std_msgs::UInt64 &msg);
        bool SensorState(sonia_common::ModemSendCmd &srv);

        void AuvStateMissionInterpreter(const bool state);
        void AuvDepthInterpreter(const float_t data);
        void AuvSyncInterpreter(const bool state);

        void MissionStateCallback(const sonia_common::ModemUpdateMissionList &msg);
        void SyncCallback(const std_msgs::Bool &msg);
        void StateMissionCallback(const std_msgs::Bool &msg);
        void DepthCallback(const std_msgs::Float32 &msg);

        bool DepthRequest(std_srvs::Empty::Request &DepthRsq, std_srvs::Empty::Response &DepthRsp);

        void Process();
        Modem_M64_t ConstructPacket(const uint64_t data);
        uint64_t DeconstructPacket(const Modem_M64_t packet);
        int8_t VerifyPacket(const Modem_M64_t packet);

        void InitMissionState(uint8_t size);
        void UpdateMissionState(uint8_t index, int8_t state);
        void UpdateMissionState_othersub(uint8_t index, int8_t state);
        void SendDepth();
        
        ros::NodeHandlePtr nh_;
        Configuration configuration_;

        ros::Subscriber underwaterComSubscriber_;
        ros::Subscriber updateMissionSubcrisber_;
        ros::Subscriber depthSubcrisber_;
        ros::Subscriber syncSubscriber_;

        ros::Publisher underwaterComPublisher_;
        ros::Publisher auvMissionPublisher_;
        ros::Publisher otherauvMissionPublisher_;
        ros::Publisher syncPublisher_;
        ros::Publisher DepthPublisher_;

        ros::ServiceClient underwaterComClient_;
        ros::ServiceServer depthSrv_;

        std::thread process_thread;
        std::mutex sensor_mutex;

        std_msgs::Bool stateMission_; 
        std_msgs::Float32 depth_;


        // Refer to read me to understand the use for it
        std_msgs::Int8MultiArray mission_state;
        std_msgs::Int8MultiArray other_sub_mission_state;
        uint8_t index_ = 0;
        uint8_t size_mission_state;
        uint8_t AUVID = 0;
        float lastDepth_;
        float other_sub_depth_;
};
}

#endif //PROC_UNDERWATER_COM_NODE