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

// function for running top launcher
void setTopLauncher(double launcherSpeed, bool sameDir) {
  if (!sameDir) {
    upTopOutTakeMotor.Set(launcherSpeed);
    bottomTopOutTakeMotor.Set(launcherSpeed);
  } else {
    upTopOutTakeMotor.Set(launcherSpeed * -1);
    bottomTopOutTakeMotor.Set(launcherSpeed);
  }
}

// auto for getting leave points.
void autoLeave() {
  if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(4000) + autoStartTime) {
    m_drive.ArcadeDrive(0, filter.Calculate(units::voltage::volt_t{0.5}).value(), true);
  } else {
    m_drive.ArcadeDrive(0, 0, true);
  }
}

void autoSpeakerLeave() {
  if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(1000) + autoStartTime) {
    setTopLauncher(1, false);
  } else if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(4000) + autoStartTime) {
    setTopLauncher(1, false);
    midOutTakeMotor.Set(0.2);
  } else if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(12000) + autoStartTime) {
    m_drive.ArcadeDrive(0, filter.Calculate(units::voltage::volt_t{-0.5}).value(), true);
    setTopLauncher(0, false);
    midOutTakeMotor.Set(0);
  } else {
    m_drive.ArcadeDrive(0, 0, true);
  }
}

void autoSpeaker() {
  if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(1000) + autoStartTime) {
    setTopLauncher(1, false);
  } else if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(4000) + autoStartTime) {
    setTopLauncher(1, false);
    midOutTakeMotor.Set(0.2);
  } else {
    setTopLauncher(0, false);
    midOutTakeMotor.Set(0);
  }
}

//intake and outtake function
void intake(double intakeMotorSpeed) {
  intakeMotorSpeed *= 2;
  // Intake
  if (intakeMotorSpeed > 1) {
    intakeMotor.Set(1);
    midOutTakeMotor.Set(0.2);
  }
  else if (intakeMotorSpeed > 0) {
    intakeMotor.Set(intakeMotorSpeed);
    midOutTakeMotor.Set(0.2);
  } else {
    intakeMotor.Set(0);
  }
}

//intake and outtake function
void outake(double outTakeMotorSpeed) {
  // Outtake
  if (outTakeMotorSpeed > 0) {
    setTopLauncher(outTakeMotorSpeed, false);
    midOutTakeMotor.Set(0.2);
  } else {
    setTopLauncher(0, false);
    midOutTakeMotor.Set(0);
  }
}

void eject(bool ejectStatus) {
   // Eject status
  if (ejectStatus) {
    setTopLauncher(-0.2, false);
    midOutTakeMotor.Set(-0.2);
    intakeMotor.Set(-0.2);
  }
}

void launcherAmp(bool enable) {
  if (enable) {
    setTopLauncher(-0.2, 0.2);
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
    // Forward and turning
  if (forwardSpd > 0 && (dir > 0.05 || dir < -0.05))
  {
    m_drive.ArcadeDrive(dir, filter.Calculate(units::voltage::volt_t{forwardSpd * -1}).value(), true);
    // Backward and turning
  }
  else if (backwardSpd > 0 && (dir > 0.05 || dir < -0.05))
  {
    m_drive.ArcadeDrive(dir, filter.Calculate(units::voltage::volt_t{backwardSpd}).value(), true);
    // Forward
  } else if (forwardSpd > 0) {
    m_drive.ArcadeDrive(0, filter.Calculate(units::voltage::volt_t{forwardSpd * -1}).value(), true);
    // Backward
  }
  else if (backwardSpd > 0)
  {
    m_drive.ArcadeDrive(0, filter.Calculate(units::voltage::volt_t{backwardSpd}).value(), true);
    // Stop
  }
  else
  {
    m_drive.ArcadeDrive(0, 0, true);
  }
}

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoDefaultLeave, kAutoDefaultLeave);
  m_chooser.AddOption(kAutoCustomSpeakerLeave, kAutoCustomSpeakerLeave);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  std::jthread visionThread(VisionThread);
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
  autoStartTime = std::chrono::high_resolution_clock::now();
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoCustomSpeakerLeave) {
    autoSpeakerLeave();
  } if (m_autoSelected == kAutoCustomSpeaker) {
    autoSpeaker();
  } else {
    autoLeave();
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoCustomSpeakerLeave) {
    autoSpeakerLeave();
  } if (m_autoSelected == kAutoCustomSpeaker) {
    autoSpeaker();
  } else {
    autoLeave();
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  //Drive controller
  /* Backwards Throttle - Left Trigger */double driveControllerLeftTrigger = driveController.GetLeftTriggerAxis();
  /* Forwards Throttle - Right Trigger */double driveControllerRightTrigger = driveController.GetRightTriggerAxis();
  /* Turning - Left Joystick*/double driveControllerLeftJoyStickX = driveController.GetLeftX();

  //Manipulator controller
  /* Speaker Outtake - Right Trigger*/double rTrigger = manipulatorController.GetRightTriggerAxis();
  /* Amplifier Outtake - Left Bumper*/ bool rBumper = manipulatorController.GetRightBumper();
  /* Intake  - Left Trigger*/double lTrigger = manipulatorController.GetLeftTriggerAxis();

  /* Eject - Left Bumber*/bool lBumper = manipulatorController.GetLeftBumper();
  
  /* Right Lifter - Right Joystick*/double rJoyStick = manipulatorController.GetRightY() * -1;
  /* Left Lifter - Left Joystick*/double lJoyStick = manipulatorController.GetLeftY() * -1;

  assistedAimSpeaker(launcherCam, CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH, GOAL_RANGE_METERS, driveController, forwardController, turnController, m_drive);
  backupDriveSystem(driveControllerRightTrigger, driveControllerLeftTrigger, driveControllerLeftJoyStickX);
  intake(lTrigger);
  outake(rTrigger);
  eject(lBumper);
  launcherAmp(rBumper);
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
