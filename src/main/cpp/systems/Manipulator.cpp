#include "systems/Manipulator.h"

#include "Constants.h"
#include "Robot.h"
#include "systems/Elevator.h"

using namespace ctre::phoenix;

void Manipulator::ExtendArm() { m_armOut = true; }

void Manipulator::RetractArm() { m_armOut = false; }

void Manipulator::SetAlgaeSpeed(double speed) { m_algaeSpeed = speed; }

void Manipulator::StartIntakingCoral() { m_coralIntaking = true; }

void Manipulator::StopIntakingCoral() { m_coralIntaking = false; }

void Manipulator::StartOutakingCoral() { m_coralOutaking = true; }

void Manipulator::StopOutakingCoral() { m_coralOutaking = false; }

void Manipulator::StartReversingCoral() { m_coralReversing = true; }

void Manipulator::StopReversingCoral() { m_coralReversing = false; }

bool Manipulator::ArmDown() const { return m_armOut; }

bool Manipulator::ElevatorSafe() const {
  if (m_firstSensor.Get() && !m_secondSensor.Get()) {
    return false;
  }

  return !m_elevatorBlockSensor.Get();
}

bool Manipulator::DoneIntaking() const {
  return m_firstSensor.Get() && m_secondSensor.Get();
}

bool Manipulator::IntakingAlgae() const {
  if (ArmDown() &&
      m_algaeSpeed == Constants::kManipulatorAlgaeManipulatorSpeed) {
    return true;
  }

  return false;
}

void Manipulator::Update(Robot::Mode mode, double t) {
  if (mode == Robot::kAuto || mode == Robot::kTeleop) {
    if (m_armOut) {
      m_algaeSolenoid.Set(frc::DoubleSolenoid::kForward);
    } else {
      m_algaeSolenoid.Set(frc::DoubleSolenoid::kReverse);
    }

    m_algaeMotor.Set(motorcontrol::TalonSRXControlMode::PercentOutput,
                     m_algaeSpeed);

    if (Elevator::GetInstance().GetPosition() >=
        Constants::kElevatorCollisionTripDistance) {
      m_coralSolenoid.Set(frc::DoubleSolenoid::kForward);
    } else {
      m_coralSolenoid.Set(frc::DoubleSolenoid::kReverse);
    }

    if (m_coralIntaking && (!m_secondSensor.Get() ||
                            (m_firstSensor.Get() && m_secondSensor.Get()))) {
      m_coralMotor.Set(motorcontrol::TalonSRXControlMode::PercentOutput,
                       Constants::kManipulatorCoralIntakeSpeed);
      m_assistMotor.Set(Constants::kManipulatorAssistOutput);
    } else if (m_coralOutaking) {
      m_coralMotor.Set(motorcontrol::TalonSRXControlMode::PercentOutput,
                       Constants::kManipulatorCoralOutakeSpeed);
      m_assistMotor.Set(0);
    } else if (!m_firstSensor.Get() && m_secondSensor.Get()) {
      m_coralMotor.Set(motorcontrol::TalonSRXControlMode::PercentOutput,
                       Constants::kManipulatorCoralSpeedReverse);
      m_assistMotor.Set(0);
    } else if (m_coralReversing) {
      m_coralMotor.Set(motorcontrol::TalonSRXControlMode::PercentOutput,
                       Constants::kManipulatorCoralRerversingSpeed);
      m_assistMotor.Set(Constants::kManipulatorAssistReverseOutput);
    } else {
      m_coralMotor.Set(motorcontrol::TalonSRXControlMode::PercentOutput, 0);
      m_assistMotor.Set(0);
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
      m_algaeMotor{Constants::kManipulatorAlgaeMotorId},
      m_assistMotor{Constants::kManipulatorAssistID,
                    rev::spark::SparkMax::MotorType::kBrushless},
      m_elevatorBlockSensor{Constants::kManipulatorElevatorBlockSensorId} {}
