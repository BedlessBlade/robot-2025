#pragma once

#include <frc/geometry/Transform3d.h>
#include <units/length.h>

class Constants {
public:
  static constexpr double kInchesPerMeter = 39.37;

  // Driverstation constants
  static constexpr int kDriverControllerId = 0;
  static constexpr int kOperatorControllerId = 1;

  // Constants for the drivetrain
  static constexpr int kPigeonCanId = 25;

  static constexpr auto kWheelBaseWidth = 24.375_in;

  static constexpr int kFlSteeringMotorId = 14;
  static constexpr int kFrSteeringMotorId = 12;
  static constexpr int kBlSteeringMotorId = 11;
  static constexpr int kBrSteeringMotorId = 17;
  static constexpr int kFlDriveMotorId = 15;
  static constexpr int kFrDriveMotorId = 16;
  static constexpr int kBlDriveMotorId = 19;
  static constexpr int kBrDriveMotorId = 13;

  static constexpr int kFlEncoderId = 24;
  static constexpr int kFrEncoderId = 23;
  static constexpr int kBlEncoderId = 22;
  static constexpr int kBrEncoderId = 21;

  static constexpr units::turn_t kEncoderOffsets[] = {
      -0.14599609375_tr, 0.33447265625_tr, 0.040283203125_tr,
      -0.080810546875_tr};

  // kS, kV, kP, kI, kD
  static constexpr std::tuple<double, double, double, double, double>
      kSteeringMotorGains{0.0, 0.0, 30.0, 0.0, 0.0};

  static constexpr double kWheelRadius = (4.0 / kInchesPerMeter) / 2; // meters

  static constexpr double kDriveCurrentLimit = 60; // Amps
  static constexpr double kDriveRampRate = 0.35;   // Seconds
  static constexpr double kDriveVelocityMultiplier = 1 / 4.65;
  static constexpr double kDriveGearRatio = 1.0 / 6.75;
  static constexpr double kDriveControlMultipler = 3.25; // meters per second
  static constexpr double kDriveAngularControlMultiplier =
      5.0;                              // radians per second
  static constexpr double kMaxV = 4.65; // meters per second

  static constexpr double kDefaultMaxV = 1.0; // meters per second
  static constexpr double kDefaultMaxW = 1.0; // radians per second

  // Vision
  static constexpr frc::Transform3d kFrontCameraTransform{
      frc::Translation3d{6.298_in, -10.622_in, 23_in},
      frc::Rotation3d{0_rad, 20_deg, 20_deg}};
  static constexpr frc::Transform3d kBackCameraTransform{
      frc::Translation3d{4.75_in, -11.25_in, 32.75_in},
      frc::Rotation3d{0_rad, 0_rad, 0_deg}};
  static constexpr auto kBlockedTags = {3, 16};

  // TalonFX ids for elevator motors.
  static constexpr int kElevatorMainMotorId = 20;
  static constexpr int kElevatorSecondMotorId = 18;
  static constexpr double kElevatorMetersPerRotation = 0.1366 / 12 * 2;

  // Values is in meters per second
  static constexpr double kElevatorCruiseVelocity =
      2.1 / kElevatorMetersPerRotation;
  // Value is in meters per second per second
  static constexpr double kElevatorAcceleration =
      8.0 / kElevatorMetersPerRotation;
  // Value in meters per second per second per second
  static constexpr double kElevatorJerk = 25.0 / kElevatorMetersPerRotation;
  // P I D CruiseVelocity kV S(overcome static friction) A(output per unit
  // of target acceleration)       Accel Jerk     kG
  static constexpr std::tuple<double, double, double, double, double, double,
                              double, double, double, double>
      kElevatorMotorGains{5,
                          0,
                          0,
                          kElevatorCruiseVelocity,
                          0.05,
                          0.01,
                          0.01,
                          kElevatorAcceleration,
                          kElevatorJerk,
                          0.2};
  // Starting offset in meters
  static constexpr double kElevatorHomePositionMeters = 0.0;
  static constexpr double kElevatorHomePositionRotations =
      kElevatorHomePositionMeters / kElevatorMetersPerRotation;
  static constexpr double kElevatorL1PositionMeters = 0;
  static constexpr double kElevatorL1PositionRotations =
      kElevatorL1PositionMeters / kElevatorMetersPerRotation;
  static constexpr double kElevatorL2PositionMeters = 0.323;
  static constexpr double kElevatorL2PositionRotations =
      kElevatorL2PositionMeters / kElevatorMetersPerRotation;
  static constexpr double kElevatorL3PositionMeters = 0.729;
  static constexpr double kElevatorL3PositionRotations =
      kElevatorL3PositionMeters / kElevatorMetersPerRotation;
  static constexpr double kElevatorL4PositionMeters = 1.32;
  static constexpr double kElevatorL4PositionRotations =
      kElevatorL4PositionMeters / kElevatorMetersPerRotation;
  static constexpr double kElevatorAlgae1PositionMeters = 0.434;
  static constexpr double kElevatorAlgae1PositionRotations =
      kElevatorAlgae1PositionMeters / kElevatorMetersPerRotation;
  static constexpr double kElevatorAlgae2PositionMeters = 0.83;
  static constexpr double kElevatorAlgae2PositionRotations =
      kElevatorAlgae2PositionMeters / kElevatorMetersPerRotation;
  static constexpr double kElevatorDefaultTolerance = 0.003;     // meters
  static constexpr double kElevatorCollisionTripDistance = 0.05; // meters
  static constexpr int kElevatorHomeSensorId = 2;
  static constexpr int kElevatorCurrentLimit = 40; // Amps

  // Manipulator
  static constexpr int kManipulatorCoralSolenoidIdForward = 14;
  static constexpr int kManipulatorCoralSolenoidIdReverse = 13;
  static constexpr int kManipulatorAlgaeSolenoidIdForward = 12;
  static constexpr int kManipulatorAlgaeSolenoidIdReverse = 11;
  static constexpr int kManipulatorFirstSensorId = 0;
  static constexpr int kManipulatorSecondSensorId = 1;
  static constexpr int kManipulatorElevatorBlockSensorId = 3;
  static constexpr int kManipulatorCoralMotorId = 10;
  static constexpr int kManipulatorAlgaeMotorId = 6;
  static constexpr double kManipulatorCoralIntakeSpeed = 0.25;
  static constexpr double kManipulatorCoralOutakeSpeed = 1.0;
  static constexpr double kManipulatorCoralSpeedReverse = -0.2;
  static constexpr double kManipulatorAlgaeManipulatorSpeed = 1.0;
  static constexpr double kManipulatorAlgaeOutakeSpeed = -1.0;
  static constexpr double kManipulatorAlgaeHoldSpeed = 0.4;
  static constexpr short kManipulatorAssistID = 26;
  static constexpr double kManipulatorAssistOutput = 0.1; //percentage of the coral neo output power for assisting intake (0 = 0% 1=100%)
  static constexpr double kManipulatorAssistReverseOutput = -0.1; //power output of the neo when scoring L1 (same as one above)
  static constexpr double kManipulatorCoralRerversingSpeed = 0.5; //pretty sure its inverted so positive

  // Locations
  static constexpr double kReefWidth = 65.2 / kInchesPerMeter;
  static constexpr double kReefOffset =
      (144 / kInchesPerMeter) + kReefWidth / 2;
  static constexpr double kFieldLength = 690.875 / kInchesPerMeter;
  static constexpr double kFieldWidth = 317 / kInchesPerMeter;
  static constexpr double kReefSpacing = 0.515; // meters
  static constexpr double kCoralSpacing = 13 / kInchesPerMeter;
  static constexpr double kStartLineOffset = (25 * 12) / kInchesPerMeter;
  static constexpr double kStartOffsetY =
      kFieldWidth / 2 - (18 / kInchesPerMeter); // meters
  static constexpr double kStartOffsetX = 0;    // meters

  // Pneumatics
  static constexpr double kMinPressure = 100; // PSI
  static constexpr double kMaxPressure = 120; // PSI

  // Path following
  static constexpr double kPathFollowingKp = 6.0;
  static constexpr double kPathFollowingKi = 0.002;
  static constexpr double kPathFollowingKd = 0.5;
  static constexpr double kPathFollowingAngleKp = 3.0;
  static constexpr double kPathFollowingAngleKi = 0;
  static constexpr double kPathFollowingAngleKd = 0.1;
  static constexpr double kPathFollowingMaxV = 1;        // meters per second
  static constexpr double kPathFollowingMaxW = 0.5;      // radians per second
  static constexpr double kPathFollowingTolerance = 0.1; // meters
  static constexpr double kPathFollowingVelocityTolerance =
      0.1; // meters per second

  static constexpr auto kFeederDistance = 24_in;
  static constexpr double kBrakeDistance = 0.25; // meters
};
