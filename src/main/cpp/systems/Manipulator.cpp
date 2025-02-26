#include "systems/Manipulator.h"

#include "Constants.h"
#include "Robot.h"
#include "systems/Elevator.h"

using namespace ctre::phoenix;

void Manipulator::ExtendArm() { m_armOut = true; }

void Manipulator::RetractArm() { m_armOut = false; }

void Manipulator::SetAlgaeSpeed(double speed) { m_algaeSpeed = speed; }

void Manipulator::StartIntakingCoral() { m_coralIntaking = true; }

void Manipulator::StopIntakingcoral() { m_coralIntaking = false; }

void Manipulator::Update(Robot::Mode mode, double t) {
  if (mode == Robot::kAuto || mode == Robot::kTeleop) {
    if (m_armOut) {
      m_algaeSolenoid.Set(frc::DoubleSolenoid::kForward);
    } else {
      m_algaeSolenoid.Set(frc::DoubleSolenoid::kReverse);
    }

    m_algaeMotor.Set(motorcontrol::TalonSRXControlMode::PercentOutput,
                     m_algaeSpeed);

    if (Elevator::GetInstance().GetPosition() > 0) {
      m_coralSolenoid.Set(frc::DoubleSolenoid::kForward);
    } else {
      m_coralSolenoid.Set(frc::DoubleSolenoid::kReverse);
    }

    if (m_coralIntaking) {
      m_coralMotor.Set(motorcontrol::TalonSRXControlMode::PercentOutput,
                       Constants::kManipulatorCoralSpeedFast);
    } else {
      m_coralMotor.Set(motorcontrol::TalonSRXControlMode::PercentOutput, 0);
    }
  }
}

Manipulator::Manipulator()
    : m_coralSolenoid{frc::PneumaticsModuleType::REVPH,
                      Constants::kManipulatorCoralSolenoidIdForward,
                      Constants::kManipulatorCoralSolenoidIdReverse},
      m_algaeSolenoid{frc::PneumaticsModuleType::REVPH,
                      Constants::kManipulatorAlgaeSolenoidIdForward,
                      Constants::kManipulatorAlgaeSolenoidIdReverse},
      m_firstSensor{Constants::kManipulatorFirstSensorId},
      m_secondSensor{Constants::kManipulatorSecondSensorId},
      m_coralMotor{Constants::kManipulatorCoralMotorId},
      m_algaeMotor{Constants::kManipulatorAlgaeMotorId} {}
