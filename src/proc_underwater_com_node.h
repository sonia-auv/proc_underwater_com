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
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <thread>
#include <mutex>

#include "Configuration.h"
#include "sonia_common/Modem_Definitions.h"
#include "sonia_common/KillSwitchMsg.h"
#include "sonia_common/MissionSwitchMsg.h"
#include "sonia_common/ModemPacket.h"

namespace proc_underwater_com {

class ProcUnderwaterComNode
{
    public:

        ProcUnderwaterComNode(const ros::NodeHandlePtr & _nh);
        ~ProcUnderwaterComNode();

        void Spin();

    private:

        void UnderwaterComInterpreterCallback(const std_msgs::String &msg);
        void SendMessage();
        bool SensorState(sonia_common::ModemPacket &srv);

        void AuvStateKillInterpreter(const bool state);
        void AuvStateMissionInterpreter(const bool state);
        void AuvDepthInterpreter(const float_t data);

        void StateKillCallback(const sonia_common::KillSwitchMsg &msg);
        void StateMissionCallback(const sonia_common::MissionSwitchMsg &msg);
        void DepthCallback(const std_msgs::Float32 &msg);

        void Verify_Link();
        
        ros::NodeHandlePtr nh_;
        Configuration configuration_;

        ros::Subscriber underwaterComSubscriber_;
        ros::Subscriber stateKillSubcrisber_;
        ros::Subscriber stateMissionSubcrisber_;
        ros::Subscriber depthSubcrisber_;

        ros::Publisher underwaterComPublisher_;
        ros::Publisher auvStateKillPublisher_;
        ros::Publisher auvStateMissionPublisher_;
        ros::Publisher auvDepthPublisher_;

        ros::ServiceClient underwaterComClient_;

        std::thread diagnostic_thread;
        std::mutex received_mutex;

        sonia_common::KillSwitchMsg stateKill_;
        sonia_common::MissionSwitchMsg stateMission_;
        std_msgs::Float32 depth_;

        bool lastStateKill_ = false;
        bool lastStateMission_ = false;
        float_t lastDepth_ = 0.0;

        char role_ = ROLE_MASTER;
        char link_ = LINK_UP;
        bool received_message_ = true;
};
}

#endif //PROC_UNDERWATER_COM_NODE