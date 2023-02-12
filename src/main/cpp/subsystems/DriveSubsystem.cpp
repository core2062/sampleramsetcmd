// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_left1{kLeftMotor1Port},
      m_left2{kLeftMotor2Port},
      m_right1{kRightMotor1Port},
      m_right2{kRightMotor2Port},
      m_odometry{m_gyro.GetRotation2d(), units::meter_t{0}, units::meter_t{0}} {
  // We need to invert one side of the drivetrain so that positive voltages
  // result in both sides moving forward. Depending on how your robot's
  // gearbox is constructed, you might have to invert the left side instead.

  m_left1.ConfigFactoryDefault();
  m_left2.ConfigFactoryDefault();
  m_right1.ConfigFactoryDefault();
  m_right2.ConfigFactoryDefault();
  // m_rightMotors.SetInverted(true);

  m_left2.Follow(m_left1);
  m_right2.Follow(m_right1);
  
  // Invert the correct side
  m_right1.SetInverted(true);
  m_right2.SetInverted(true);

  // configure the sensors
  m_left1.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,0, 10);
  m_right1.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,0, 10);

  // set the sensor value to track the forward (pos) and reverise(neg) of the controller
  m_right1.SetSensorPhase(true);
  m_left1.SetSensorPhase(true);

  // Set the distance per pulse for the encoders
  // This is handled by NativeUnitsToDistanceMeters
  // m_leftEncoder.SetDistancePerPulse(kEncoderDistancePerPulse);
  // m_rightEncoder.SetDistancePerPulse(kEncoderDistancePerPulse);

  ResetEncoders();
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(m_gyro.GetRotation2d(),
                  NativeUnitsToDistanceMeters(m_left1.GetSelectedSensorPosition(0)),
                  NativeUnitsToDistanceMeters(m_right2.GetSelectedSensorPosition(0)));
}

void DriveSubsystem::ArcadeDrive(double fwd, double rot) {
  m_drive.ArcadeDrive(fwd, rot);
}

void DriveSubsystem::TankDriveVolts(units::volt_t left, units::volt_t right) {
  m_leftMotors.SetVoltage(left);
  m_rightMotors.SetVoltage(right);
  m_drive.Feed();
}

void DriveSubsystem::ResetEncoders() {
  // TODO: reset encoders
  // m_leftEncoder.Reset();
  // m_rightEncoder.Reset();
}

double DriveSubsystem::GetAverageEncoderDistance() {
  return (static_cast<double>(NativeUnitsToDistanceMeters(m_left1.GetSelectedSensorPosition()) + NativeUnitsToDistanceMeters(m_right1.GetSelectedSensorPosition())) / 2.0);
}

// frc::Encoder& DriveSubsystem::GetLeftEncoder() {
//   return m_leftEncoder;
// }

// frc::Encoder& DriveSubsystem::GetRightEncoder() {
//   return m_rightEncoder;
// }

void DriveSubsystem::SetMaxOutput(double maxOutput) {
  m_drive.SetMaxOutput(maxOutput);
}

units::degree_t DriveSubsystem::GetHeading() const {
  return m_gyro.GetRotation2d().Degrees();
}

// double DriveSubsystem::GetTurnRate() {
//   return -m_gyro.GetRate();
// }

frc::Pose2d DriveSubsystem::GetPose() {
  return m_odometry.GetPose();
}

frc::DifferentialDriveWheelSpeeds DriveSubsystem::GetWheelSpeeds() {
  return {units::meters_per_second_t{NativeUnitsToDistanceMeters(m_left1.GetSelectedSensorVelocity(0))/ 0.100_s},
          units::meters_per_second_t{NativeUnitsToDistanceMeters(m_right1.GetSelectedSensorVelocity(0))/0.100_s}};
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  ResetEncoders();
  m_odometry.ResetPosition(m_gyro.GetRotation2d(),
                           NativeUnitsToDistanceMeters(m_left1.GetSelectedSensorPosition(0)),
                           NativeUnitsToDistanceMeters(m_right1.GetSelectedSensorPosition(0)), pose);
}

units::meter_t DriveSubsystem::NativeUnitsToDistanceMeters(double sensorCounts){
	double motorRotations = (double)sensorCounts / kEncoderCPR;
	double wheelRotations = motorRotations / kGearRatio;
	units::meter_t position = units::meter_t{wheelRotations * (std::numbers::pi * kWheelDiameter)};
	return position;
}
