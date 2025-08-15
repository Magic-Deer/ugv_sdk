/*
 * Tracer_robot.cpp
 *
 * Created on: Jul 14, 2021 23:14
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "ugv_sdk/mobile_robot/tracer_robot.hpp"
#include "ugv_sdk/details/robot_base/tracer_base.hpp"

namespace westonrobot {
TracerRobot::TracerRobot(ProtocolVersion protocol) {
  robot_ = new TracerBase();
}

TracerRobot::~TracerRobot() {
  if (robot_) delete robot_;
}

std::string TracerRobot::RequestVersion(int timeout_sec) {
    return robot_->RequestVersion(timeout_sec);
}

void TracerRobot::EnableCommandedMode() { robot_->EnableCommandedMode(); }

bool TracerRobot::Connect(std::string can_name) {
  return robot_->Connect(can_name);
}

void TracerRobot::ResetRobotState() { robot_->ResetRobotState(); }

ProtocolVersion TracerRobot::GetParserProtocolVersion() {
  return robot_->GetParserProtocolVersion();
}

void TracerRobot::SetMotionCommand(double linear_vel, double angular_vel) {
  auto Tracer = dynamic_cast<TracerInterface*>(robot_);
  Tracer->SetMotionCommand(linear_vel, angular_vel);
}
void TracerRobot::SetLightCommand(AgxLightMode f_mode, uint8_t f_value) {
  auto scout = dynamic_cast<TracerInterface*>(robot_);
  scout->SetLightCommand(f_mode, f_value);
}

void TracerRobot::DisableLightControl() {
  auto tracer = dynamic_cast<TracerInterface*>(robot_);
  tracer->DisableLightControl();
}

TracerCoreState TracerRobot::GetRobotState() {
  auto Tracer = dynamic_cast<TracerInterface*>(robot_);
  return Tracer->GetRobotState();
}

TracerActuatorState TracerRobot::GetActuatorState() {
  auto Tracer = dynamic_cast<TracerInterface*>(robot_);
  return Tracer->GetActuatorState();
}

TracerCommonSensorState TracerRobot::GetCommonSensorState() {
  auto scout = dynamic_cast<TracerInterface*>(robot_);
  return scout->GetCommonSensorState();
}

}  // namespace westonrobot