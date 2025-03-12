#include "systems/SwerveDrive.h"

#include "Constants.h"
#include "Robot.h"
#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "units/velocity.h"

// We need to initialize the gyro and kinematics members. The kinematics
// constructor needs the positions of the four wheels. The coordinate system is
// +x is towards the front of the robot, and +y is to the robot's left.
SwerveDrive::SwerveDrive()
    : m_gyro{Constants::kPigeonCanId},
      m_kinematics{frc::Translation2d{Constants::kWheelBaseWidth / 2,
                                      Constants::kWheelBaseWidth / 2},
                   frc::Translation2d{Constants::kWheelBaseWidth / 2,
                                      -Constants::kWheelBaseWidth / 2},
                   frc::Translation2d{-Constants::kWheelBaseWidth / 2,
                                      Constants::kWheelBaseWidth / 2},
                   frc::Translation2d{-Constants::kWheelBaseWidth / 2,
                                      -Constants::kWheelBaseWidth / 2}},
      m_steeringMotors{{Constants::kFlSteeringMotorId},
                       {Constants::kFrSteeringMotorId},
                       {Constants::kBlSteeringMotorId},
                       {Constants::kBrSteeringMotorId}},
      m_driveMotors{{Constants::kFlDriveMotorId},
                    {Constants::kFrDriveMotorId},
                    {Constants::kBlDriveMotorId},
                    {Constants::kBrDriveMotorId}},
      m_encoders{{Constants::kFlEncoderId},
                 {Constants::kFrEncoderId},
                 {Constants::kBlEncoderId},
                 {Constants::kBrEncoderId}},
      m_poseEstimator{
          m_kinematics,
          GetGyroRotation2d(),
          {frc::SwerveModulePosition{
               units::meter_t{
                   m_driveMotors[0].GetPosition().GetValue().value() * 2 *
                   M_PI * Constants::kWheelRadius * Constants::kDriveGearRatio},
               frc::Rotation2d{m_encoders[0].GetPosition().GetValue() * 2 *
                               M_PI}},
           frc::SwerveModulePosition{
               units::meter_t{
                   m_driveMotors[1].GetPosition().GetValue().value() * 2 *
                   M_PI * Constants::kWheelRadius * Constants::kDriveGearRatio},
               frc::Rotation2d{m_encoders[1].GetPosition().GetValue() * 2 *
                               M_PI}},
           frc::SwerveModulePosition{
               units::meter_t{
                   m_driveMotors[2].GetPosition().GetValue().value() * 2 *
                   M_PI * Constants::kWheelRadius * Constants::kDriveGearRatio},
               frc::Rotation2d{m_encoders[2].GetPosition().GetValue() * 2 *
                               M_PI}},
           frc::SwerveModulePosition{
               units::meter_t{
                   m_driveMotors[3].GetPosition().GetValue().value() * 2 *
                   M_PI * Constants::kWheelRadius * Constants::kDriveGearRatio},
               frc::Rotation2d{m_encoders[3].GetPosition().GetValue() * 2 *
                               M_PI}}},
          frc::Pose2d{}} {
  // Configure the PID values for the position mode on the steering motors
  auto [kS, kV, kP, kI, kD] = Constants::kSteeringMotorGains;
  configs::Slot0Configs config;
  config.kS = kS;
  config.kV = kV;
  config.kP = kP;
  config.kI = kI;
  config.kD = kD;

  // Create a current limit config to apply to the drive motors
  auto currentLimitConfig = configs::CurrentLimitsConfigs{}
                                .WithSupplyCurrentLimitEnable(true)
                                .WithSupplyCurrentLimit(units::ampere_t{
                                    Constants::kDriveCurrentLimit});

  // Apply an open loop ramp rate to the drive motors only
  EnableRamp();

  for (int i = 0; i < 4; i++) {
    // At the same time, go ahead and configure the remote sensor to be the
    // CANCoder.
    m_encoders[i].GetConfigurator().Apply(
        configs::MagnetSensorConfigs{}
            .WithSensorDirection(
                signals::SensorDirectionValue::CounterClockwise_Positive)
            .WithMagnetOffset(Constants::kEncoderOffsets[i]));
    m_steeringMotors[i].GetConfigurator().Apply(
        configs::TalonFXConfiguration{}
            .WithSlot0(config)
            .WithFeedback(
                configs::FeedbackConfigs{}.WithRemoteCANcoder(m_encoders[i]))
            .WithMotorOutput(configs::MotorOutputConfigs{}.WithInverted(true)));

    m_driveMotors[i].GetConfigurator().Apply(currentLimitConfig);

    // Steering motors should always be in coast mode because we are using
    // closed-loop control
    m_steeringMotors[i].SetNeutralMode(signals::NeutralModeValue::Coast);
  }

  // Default to coast mode
  Coast();
}

// This function needs to be called by the looper to update the drive motors
void SwerveDrive::Update(Robot::Mode mode, double t) {
  // Update the estimation of where the robot thinks it is on the field
  m_poseEstimator.Update(
      GetGyroRotation2d(),
      {frc::SwerveModulePosition{
           units::meter_t{m_driveMotors[0].GetPosition().GetValue().value() *
                          2 * M_PI * Constants::kWheelRadius *
                          Constants::kDriveGearRatio},
           m_encoders[0].GetPosition().GetValue()},
       frc::SwerveModulePosition{
           units::meter_t{m_driveMotors[1].GetPosition().GetValue().value() *
                          2 * M_PI * Constants::kWheelRadius *
                          Constants::kDriveGearRatio},
           m_encoders[1].GetPosition().GetValue()},
       frc::SwerveModulePosition{
           units::meter_t{m_driveMotors[2].GetPosition().GetValue().value() *
                          2 * M_PI * Constants::kWheelRadius *
                          Constants::kDriveGearRatio},
           m_encoders[2].GetPosition().GetValue()},
       frc::SwerveModulePosition{
           units::meter_t{m_driveMotors[3].GetPosition().GetValue().value() *
                          2 * M_PI * Constants::kWheelRadius *
                          Constants::kDriveGearRatio},
           m_encoders[3].GetPosition().GetValue()}});

  if (mode == Robot::kAuto || mode == Robot::kTeleop) {
    // Use the WPILib kinematics class to determine the individual wheel angles
    // and velocities.
    auto speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        units::meters_per_second_t{m_vx}, units::meters_per_second_t{m_vy},
        units::radians_per_second_t{m_w}, GetPose2d().Rotation());
    auto states = m_kinematics.ToSwerveModuleStates(speeds);

    // Prevent velocities from clipping
    frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(
        &states, units::meters_per_second_t{Constants::kMaxV});
    auto [fl, fr, bl, br] = states;

    // Optimize the angle setpoints to make the wheels reach the correct angle
    // as fast as possible (not go the long way around).
    fl.Optimize(m_encoders[0].GetPosition().GetValue());
    fr.Optimize(m_encoders[1].GetPosition().GetValue());
    bl.Optimize(m_encoders[2].GetPosition().GetValue());
    br.Optimize(m_encoders[3].GetPosition().GetValue());

    // Decrease the speed of modules that aren't pointing in the correct
    // direction.
    fl.speed *= (fl.angle - frc::Rotation2d{units::radian_t{
                                m_encoders[0].GetPosition().GetValue()}})
                    .Cos();
    fr.speed *= (fr.angle - frc::Rotation2d{units::radian_t{
                                m_encoders[1].GetPosition().GetValue()}})
                    .Cos();
    bl.speed *= (bl.angle - frc::Rotation2d{units::radian_t{
                                m_encoders[2].GetPosition().GetValue()}})
                    .Cos();
    br.speed *= (br.angle - frc::Rotation2d{units::radian_t{
                                m_encoders[3].GetPosition().GetValue()}})
                    .Cos();

    // Set the positions for the wheel angles
    m_steeringMotors[0].SetControl(controls::PositionVoltage{
        units::turn_t{fl.angle.Radians().value() / 2 /
                      M_PI}}.WithSlot(0));
    m_steeringMotors[1].SetControl(controls::PositionVoltage{
        units::turn_t{fr.angle.Radians().value() / 2 /
                      M_PI}}.WithSlot(0));
    m_steeringMotors[2].SetControl(controls::PositionVoltage{
        units::turn_t{bl.angle.Radians().value() / 2 /
                      M_PI}}.WithSlot(0));
    m_steeringMotors[3].SetControl(controls::PositionVoltage{
        units::turn_t{br.angle.Radians().value() / 2 /
                      M_PI}}.WithSlot(0));

    // Use open loop control on the drive motors to get close enough
    // Using closed-loop velocity control with CTRE devices at lower speeds can
    // cause jitter.
    m_driveMotors[0].SetControl(controls::DutyCycleOut{
        fl.speed.value() * Constants::kDriveVelocityMultiplier});
    m_driveMotors[1].SetControl(controls::DutyCycleOut{
        fr.speed.value() * Constants::kDriveVelocityMultiplier});
    m_driveMotors[2].SetControl(controls::DutyCycleOut{
        bl.speed.value() * Constants::kDriveVelocityMultiplier});
    m_driveMotors[3].SetControl(controls::DutyCycleOut{
        br.speed.value() * Constants::kDriveVelocityMultiplier});
  }
}

frc::Rotation2d SwerveDrive::GetGyroRotation2d() const {
  return m_gyro.GetRotation2d();
}

frc::Pose2d SwerveDrive::GetPose2d() const {
  return m_poseEstimator.GetEstimatedPosition();
}

void SwerveDrive::Coast() {
  for (int i = 0; i < 4; i++) {
    m_driveMotors[i].SetNeutralMode(signals::NeutralModeValue::Coast);
  }
}

void SwerveDrive::Brake() {
  for (int i = 0; i < 4; i++) {
    m_driveMotors[i].SetNeutralMode(signals::NeutralModeValue::Brake);
  }
}

// Sets the drive velocity in meters per second and radians per second
// Call this function with no arguments to stop the robot
void SwerveDrive::DriveVelocity(double vx, double vy, double w) {
  m_vx = vx;
  m_vy = vy;
  m_w = w;
}

void SwerveDrive::ResetPose(frc::Pose2d pose) {
  m_poseEstimator.ResetPose(pose);
}

void SwerveDrive::VisionUpdate(frc::Pose2d pose, units::second_t timestamp) {
  m_poseEstimator.AddVisionMeasurement(pose, timestamp);
}

void SwerveDrive::EnableRamp() {
  auto rampRateConfig =
      configs::OpenLoopRampsConfigs{}.WithVoltageOpenLoopRampPeriod(
          units::second_t{Constants::kDriveRampRate});
  for (int i = 0; i < 4; i++) {
    m_driveMotors[i].GetConfigurator().Apply(rampRateConfig);
  }
}

void SwerveDrive::DisableRamp() {
  auto rampRateConfig =
      configs::OpenLoopRampsConfigs{}.WithVoltageOpenLoopRampPeriod(
          units::second_t{0});
  for (int i = 0; i < 4; i++) {
    m_driveMotors[i].GetConfigurator().Apply(rampRateConfig);
  }
}

double SwerveDrive::VelocityMagnitude() {
  return std::sqrt(m_vx * m_vx + m_vy * m_vy);
}
