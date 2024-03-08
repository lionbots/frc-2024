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
#include <chrono>

// Used for auto (time based)
auto autoStartTime = std::chrono::high_resolution_clock::now();

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

//Motor Controller For Intake
rev::CANSparkMax intakeMoter(8, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
rev::CANSparkMax upTopOutTakeMotor(7, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
rev::CANSparkMax bottomTopOutTakeMotor(6, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
rev::CANSparkMax midOutTakeMotor{5, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

//Motors Controller For Lifter
ctre::phoenix::motorcontrol::can::TalonSRX rLiftMotor{1};
ctre::phoenix::motorcontrol::can::TalonSRX lLiftMotor{1};

//XBox Controller
frc::XboxController manipulatorController(0);

// auto for getting leave points.
void autoLeave() {
  if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(4000) + autoStartTime) {
    d_drive.ArcadeDrive(0, 0.5, true);
  } else {
    d_drive.ArcadeDrive(0, 0.0, true);
  }
}

//intake and outtake function
void launcher(double outTakeMotorSpeed, double intakeMotorSpeed, bool ejectStatus) {
  // Intake
  if (intakeMotorSpeed > 1) {
    intakeMoter.Set(1);
  }
  else if (intakeMotorSpeed > 0) {
    intakeMoter.Set(intakeMotorSpeed);
  }
  // Outtake
  if (outTakeMotorSpeed > 0) {
    upTopOutTakeMotor.Set(outTakeMotorSpeed);
    midOutTakeMotor.Set(0.2);
    bottomTopOutTakeMotor.Set(outTakeMotorSpeed);
  }
  // Eject status
  if (ejectStatus == true) {
    upTopOutTakeMotor.Set(-0.2);
    bottomTopOutTakeMotor.Set(-0.2);
    midOutTakeMotor.Set(-0.2);
  } 
}

//lifter function
void lifter(double rSideSpeed, double lSideSpeed) {
  // Right Lifter
  if (rSideSpeed > 0.05) {
    rLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, rSideSpeed);
  }
  else if (rSideSpeed < -0.05) {
    rLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, rSideSpeed * -1);
  }
  else {
    rLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 0);
  }
  // Left Lifter
  if (lSideSpeed > 0.05) {
    lLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, lSideSpeed);
  }
  else if (lSideSpeed < -0.05) {
    lLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, lSideSpeed * -1);
  }
  else {
    lLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 0);
  }
}

void backupDriveSystem(double forwardSpd, double backwardSpd, double dir){
  if (forwardSpd > 0 && (dir > 0.05 || dir < -0.05))
  {
    d_drive.ArcadeDrive(dir, forwardSpd * -1, true);
    // Backward and turning
  }
  else if (dir > 0 && (dir > 0.05 || dir < -0.05))
  {
    d_drive.ArcadeDrive(dir, backwardSpd, true);
    // Forward
  } else if (forwardSpd > 0)
  {
    d_drive.ArcadeDrive(0, forwardSpd  * -1, true);
    // Backward
  }
  else if (backwardSpd > 0)
  {
    d_drive.ArcadeDrive(0, backwardSpd, true);
    // Stop
  }
  else
  {
    d_drive.ArcadeDrive(0, 0, true);
  }
}

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
  double driveControllerLeftJoyStickX = driveController.GetLeftX();

  double rTrigger = manipulatorController.GetRightTriggerAxis();
  double lTrigger = manipulatorController.GetLeftTriggerAxis();

  bool lBumper = manipulatorController.GetLeftBumper();
  
  double rJoyStick = manipulatorController.GetRightY() * -1;
  double lJoyStick = manipulatorController.GetLeftY() * -1;

  backupDriveSystem(driveControllerRightTrigger, driveControllerLeftTrigger, driveControllerLeftJoyStickX);
  launcher(rTrigger, lTrigger, lBumper);
  lifter(rJoyStick, lJoyStick);
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
