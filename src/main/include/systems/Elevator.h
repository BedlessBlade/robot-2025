#pragma once

#include <Constants.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DigitalInput.h>

#include "Robot.h"

using namespace ctre::phoenix6;

class Elevator {
public:
  enum Position { kHome, kL1, kL2, kL3, kL4, kAlgae1, kAlgae2 };

  static Elevator &GetInstance() {
    static Elevator instance;
    return instance;
  }

  void SetPosition(Position position);
  void Update(Robot::Mode mode, double t);
  double GetPosition();
  double GetError();
  void Zero();
  void Brake();
  void Coast();

private:
  hardware::TalonFX m_mainMotor{Constants::kElevatorMainMotorId};
  hardware::TalonFX m_secondMotor{Constants::kElevatorSecondMotorId};

  Position m_position;

  frc::DigitalInput m_homeSwitch;

  bool m_homeSwitchLastState = false;

  Elevator();
};
