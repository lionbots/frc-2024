// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/XboxController.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <rev/SparkMaxAbsoluteEncoder.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>

//Motor controller for Drive system
rev::CANSparkMax frMotor{2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
rev::CANSparkMax flMotor{1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
rev::CANSparkMax brMotor{3, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
rev::CANSparkMax blMotor{4, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
frc::MotorControllerGroup lMotorGroup(flMotor,blMotor);
frc::MotorControllerGroup rMotorGroup(frMotor,brMotor);

//Differentialdrive Object
frc::DifferentialDrive d_drive{lMotorGroup, rMotorGroup};

//XBox Controller
frc::XboxController driveController(5);

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  //Drive controller
  double driveControllerLeftTrigger = driveController.GetLeftTriggerAxis();
  double driveControllerRightTrigger = driveController.GetRightTriggerAxis();
  double driveControllerRightJoyStickX = driveController.GetRightX();

  // Forward and turning
  if (driveControllerRightTrigger > 0 && (driveControllerRightJoyStickX > 0.05 || driveControllerRightJoyStickX < -0.05))
  {
    d_drive.ArcadeDrive(driveControllerRightJoyStickX, driveControllerRightTrigger * -1, true);
    // Backward and turning
  }
  else if (driveControllerLeftTrigger > 0 && (driveControllerRightJoyStickX > 0.05 || driveControllerRightJoyStickX < -0.05))
  {
    d_drive.ArcadeDrive(driveControllerRightJoyStickX, driveControllerLeftTrigger, true);
    // Forward
  } else if (driveControllerRightTrigger > 0)
  {
    d_drive.ArcadeDrive(0, driveControllerRightTrigger  * -1, true);
    // Backward
  }
  else if (driveControllerLeftTrigger > 0)
  {
    d_drive.ArcadeDrive(0, driveControllerLeftTrigger, true);
    // Stop
  }
  else
  {
    d_drive.ArcadeDrive(0, 0, true);
  }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
