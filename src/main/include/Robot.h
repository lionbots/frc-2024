// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/XboxController.h>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkLowLevel.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>

#include <frc/controller/PIDController.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <frc/ADIS16470_IMU.h>
#include <photon/PhotonUtils.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

#include <frc/estimator/PoseEstimator.h>
#include <frc/estimator/DifferentialDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <rev/SparkAbsoluteEncoder.h>

#include <units/angle.h>
#include <units/length.h>
#include <units/base.h>
#include <frc/filter/SlewRateLimiter.h>

#include <thread>
#include <cscore_oo.h>
#include <cameraserver/CameraServer.h>

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoDefaultLeave = "Leave";
  const std::string kAutoCustomSpeaker = "Speaker";
  const std::string kAutoCustomSpeakerLeave = "Speaker + Leave";
  std::string m_autoSelected;
  static void VisionThread()
  {
    cs::UsbCamera cameraF{"FrontCamera", 0};
    cs::UsbCamera cameraB{"BackCamera", 1};
    cameraF.SetResolution(320, 240);
    cameraB.SetResolution(160, 120);
    cameraF.SetFPS(30);
    cameraB.SetFPS(20);
    cameraF = frc::CameraServer::StartAutomaticCapture(0);
    cameraB = frc::CameraServer::StartAutomaticCapture(1);
    cs::CvSink cvSink = frc::CameraServer::GetVideo();
    cs::CvSource outputStream = frc::CameraServer::PutVideo("Rectangle", 800, 700);
  }
  void assistedAimSpeaker(photon::PhotonCamera &launcherCam, const units::length::meter_t &CAMERA_HEIGHT, const units::length::meter_t &TARGET_HEIGHT, const units::angle::radian_t &CAMERA_PITCH, const units::meter_t &GOAL_RANGE_METERS, frc::XboxController &xboxController, frc::PIDController &forwardController, frc::PIDController &turnController, frc::DifferentialDrive &m_drive)
  {
    // speed of which we move forward
    double forwardSpeed;
    // speed of which we move turn
    double rotationSpeed;

    bool aButtonVal = xboxController.GetAButton();
    const auto &result = launcherCam.GetLatestResult();
    if (result.HasTargets() && aButtonVal == 1)
    {
      // First calculate range
      units::meter_t range = photon::PhotonUtils::CalculateDistanceToTarget(CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH, units::degree_t{result.GetBestTarget().GetPitch()});

      // Use this range as the measurement we give to the PID controller.

      // -1.0 required to ensure positive PID controller effort _increases_

      // range

      forwardSpeed = -forwardController.Calculate(range.value(),

                                                  GOAL_RANGE_METERS.value());

      // Also calculate angular power

      // -1.0 required to ensure positive PID controller effort _increases_ yaw

      rotationSpeed =

          -turnController.Calculate(result.GetBestTarget().GetYaw(), 0);

      // TODO: run motor based on PID values
      m_drive.ArcadeDrive(rotationSpeed, forwardSpeed, true);
    }
    else
    {
      /* MAJOR TODO: Figure out what to do if target can not be seen. I am thinking some PID setpoint aand motor calls*/
    }
  }
};

// Drive motors
rev::CANSparkMax frMotor{2, rev::CANSparkLowLevel::MotorType::kBrushless};
rev::CANSparkMax flMotor{1, rev::CANSparkLowLevel::MotorType::kBrushless};
rev::CANSparkMax brMotor{3, rev::CANSparkLowLevel::MotorType::kBrushless};
rev::CANSparkMax blMotor{4, rev::CANSparkLowLevel::MotorType::kBrushless};

frc::MotorControllerGroup lMotorGroup{flMotor, blMotor};
frc::MotorControllerGroup rMotorGroup{frMotor, brMotor};

frc::DifferentialDrive m_drive{lMotorGroup, rMotorGroup};

// Motor Controller For Intake
rev::CANSparkMax intakeMotor{8, rev::CANSparkLowLevel::MotorType::kBrushless};
// Motor Controller For Outake/Launcher
rev::CANSparkMax upTopOutTakeMotor{7, rev::CANSparkLowLevel::MotorType::kBrushless};
rev::CANSparkMax bottomTopOutTakeMotor{6, rev::CANSparkLowLevel::MotorType::kBrushless};
rev::CANSparkMax midOutTakeMotor{5, rev::CANSparkLowLevel::MotorType::kBrushless};

// Motors Controller For Lifter
ctre::phoenix::motorcontrol::can::TalonSRX rLiftMotor{1};
ctre::phoenix::motorcontrol::can::TalonSRX lLiftMotor{1};

// XBox Controller
frc::XboxController driveController{5};
frc::XboxController manipulatorController(0);

// Slew rate limiter
frc::SlewRateLimiter<units::volts> filter{2_V / 0.5_s};

// Photonvision related
photon::PhotonCamera launcherCam{"Launcher Cam"};
frc::AprilTagFieldLayout aprilTagFieldLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);
frc::Transform3d robotToCamera;
// Populate with empty values, I belive this template values make it so that the start pioint is 0,0,0,0 degree
frc::Pose3d robotPose{frc::Translation3d(0_m, 0_m, 0_m), frc::Rotation3d(0_rad, 0_rad, 0_rad)};
photon::PhotonPoseEstimator photonPoseEstimator{aprilTagFieldLayout, photon::PoseStrategy::LOWEST_AMBIGUITY, robotToCamera};

// Drive encoders
rev::SparkAbsoluteEncoder lEncoder = flMotor.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);
rev::SparkAbsoluteEncoder rEncoder = frMotor.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);

// Instanciaied for contructing robot postEstimator
// Tranforms rotation into meters traveled
//
double lDistance = lEncoder.GetPosition() * 18.8495559215;
double rDistance = rEncoder.GetPosition() * 18.8495559215;

const units::meter_t CAMERA_HEIGHT = 24_in;

const units::meter_t TARGET_HEIGHT = 5_ft;

// Angle between horizontal and the camera.

const units::radian_t CAMERA_PITCH = 0_deg;

// How far from the target we want to be

const units::meter_t GOAL_RANGE_METERS = 3_ft;

// PID constants should be tuned per robot

const double LINEAR_P = 0.1;

const double LINEAR_D = 0.0;

frc::PIDController forwardController{LINEAR_P, 0.0, LINEAR_D};

const double ANGULAR_P = 0.1;

const double ANGULAR_D = 0.0;

frc::PIDController turnController{ANGULAR_P, 0.0, ANGULAR_D};

// Gyroscope
frc::ADIS16470_IMU gyroscope{};
// Declaration for gyroscope
units::degree_t angle = gyroscope.GetAngle();

frc::DifferentialDriveKinematics dDKinematic{units::meter_t{0.530225}};
// Pose Estimator
frc::DifferentialDrivePoseEstimator robotPoseEstimator{dDKinematic, units::degree_t{gyroscope.GetAngle()}, units::meter_t{lDistance}, units::meter_t{rDistance}, frc::Pose2d{}};