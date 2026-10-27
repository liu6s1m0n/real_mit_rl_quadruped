/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "legged_interface/SwitchedModelReferenceManager.h"

namespace ocs2 {
namespace legged_robot {


std::vector<scalar_t> stance_times{ 0.0, 0.5 };
std::vector<size_t> stance_modes{ 15 };
ModeSequenceTemplate stance(stance_times, stance_modes);

std::vector<scalar_t> trot_times{ 0.0, 0.3, 0.6 };
std::vector<size_t> trot_modes{ 9, 6 };
ModeSequenceTemplate trot(trot_times, trot_modes);

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelReferenceManager::SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                             std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr)
    : ReferenceManager(TargetTrajectories(), ModeSchedule()),
      gaitSchedulePtr_(std::move(gaitSchedulePtr)),
      swingTrajectoryPtr_(std::move(swingTrajectoryPtr)) {}

void SwitchedModelReferenceManager::attachRosNode(const rclcpp::Node::SharedPtr& nh) {
        auto MysetTrotCallback = [this](const std_msgs::msg::Float32::ConstSharedPtr& msg) {
        MysetTrotFlag = true;
        MysetStanceFlag= false;
      };
      MysetTrotSub_ = nh->create_subscription<std_msgs::msg::Float32>("/set_trot", 1, MysetTrotCallback);

      auto MysetStanceCallback = [this](const std_msgs::msg::Float32::ConstSharedPtr& msg) {
         MysetTrotFlag = false;
          MysetStanceFlag= true;
      };
      MysetStanceSub_ = nh->create_subscription<std_msgs::msg::Float32>("/set_stance", 1, MysetStanceCallback);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::setModeSchedule(const ModeSchedule& modeSchedule) {
  ReferenceManager::setModeSchedule(modeSchedule);
  gaitSchedulePtr_->setModeSchedule(modeSchedule);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
contact_flag_t SwitchedModelReferenceManager::getContactFlags(scalar_t time) const {
  return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                     TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  myGait(initTime, finalTime, modeSchedule);

  const scalar_t terrainHeight = 0.0;
  swingTrajectoryPtr_->update(modeSchedule, terrainHeight);
}

scalar_t SwitchedModelReferenceManager::findInsertModeSequenceTemplateTimer(ModeSchedule& modeSchedule,
                                                                            scalar_t current_time)
{
  auto& modeSequence = modeSchedule.modeSequence;
  auto& eventTimes = modeSchedule.eventTimes;

  const auto time_insert_it = std::lower_bound(eventTimes.begin(), eventTimes.end(), current_time);
  const size_t id = std::distance(eventTimes.begin(), time_insert_it);

  return eventTimes[id];
}

void SwitchedModelReferenceManager::myGait( scalar_t initTime, scalar_t finalTime,
                                             ModeSchedule& modeSchedule)
{

      if(MysetStanceFlag == true&&MysetTrotFlag== false)
      {
        MysetStanceFlag = false;
        MysetTrotFlag = false;
        printf("start to stance\n");
        auto inserTimer = findInsertModeSequenceTemplateTimer(modeSchedule, initTime);
        gaitSchedulePtr_->insertModeSequenceTemplate(stance, inserTimer, finalTime);

      }
      else if(MysetStanceFlag == false&&MysetTrotFlag == true)
      {
        MysetStanceFlag = false;
        MysetTrotFlag = false;
        printf("start to trot\n");
        auto inserTimer = findInsertModeSequenceTemplateTimer(modeSchedule, initTime);
        gaitSchedulePtr_->insertModeSequenceTemplate(trot, inserTimer, finalTime);

      }
}

}  // namespace legged_robot
}  // namespace ocs2
