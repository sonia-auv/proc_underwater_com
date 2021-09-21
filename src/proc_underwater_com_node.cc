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
    }

    // Node Destructor
    ProcUnderwaterComNode::~ProcUnderwaterComNode(){}

    // Spin
    void ProcUnderwaterComNode::Spin()
    {
        ros::Rate r(1); // 1 Hz

        while(ros::ok())
        {
            ROS_INFO_STREAM("Node is initialised");
            ros::spinOnce();
            r.sleep();
        }
    }

    void ProcUnderwaterComNode::UnderwaterComInterpreterCallback(const std_msgs::String &msg)
    {

    }

    void ProcUnderwaterComNode::StateKillCallback(const sonia_common::KillSwitchMsg &msg)
    {

    }

    void ProcUnderwaterComNode::StateMissionCallback(const sonia_common::MissionSwitchMsg &msg)
    {
        
    }
}