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
#include <frc/filter/SlewRateLimiter.h>

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
ctre::phoenix::motorcontrol::can::TalonSRX intakeMotor {8};
rev::CANSparkMax upTopOutTakeMotor(7, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
rev::CANSparkMax bottomTopOutTakeMotor(6, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
rev::CANSparkMax midOutTakeMotor{5, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

//Motors Controller For Lifter
ctre::phoenix::motorcontrol::can::TalonSRX rLiftMotor{9};
ctre::phoenix::motorcontrol::can::TalonSRX lLiftMotor{10};

//XBox Controller
frc::XboxController manipulatorController(0);

// Slew rate limiter
// frc::SlewRateLimiter<units::volts> filter{2_V / 0.5_s};

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
    d_drive.ArcadeDrive(0, /* filter.Calculate(units::voltage::volt_t{-0.5}).value() */ -0.5, true);
  } else {
    d_drive.ArcadeDrive(0, 0, true);
  }
}

void autoSpeakerLeave() {
  if(std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(350) + autoStartTime) {
    d_drive.ArcadeDrive(0, 0.5, true);
  } else if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(1100) + autoStartTime) {
    setTopLauncher(1, false);
  } else if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(4000) + autoStartTime) {
    setTopLauncher(1, false);
    midOutTakeMotor.Set(0.2);
  } else if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(12000) + autoStartTime) {
    d_drive.ArcadeDrive(0, /* filter.Calculate(units::voltage::volt_t{0.5}).value() */ 0.5, true);
    setTopLauncher(0, false);
    midOutTakeMotor.Set(0);
  } else {
    d_drive.ArcadeDrive(0, 0, true);
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

void autoDelayedSpeaker() {
  if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(7000) + autoStartTime) {
    d_drive.ArcadeDrive(0, 0, false);
  } else if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(7350) + autoStartTime) {
    d_drive.ArcadeDrive(0, 0.5, false);
  } else if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(8350) + autoStartTime) {
    setTopLauncher(1, false); 
  } else if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(11350) + autoStartTime) {
    setTopLauncher(1, false);
    midOutTakeMotor.Set(0.2);
  } else if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(11700) + autoStartTime) {
    d_drive.ArcadeDrive(0, -0.5, false);
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
    intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 1);
    midOutTakeMotor.Set(0.2);
  }
  else if (intakeMotorSpeed > 0) {
    intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, intakeMotorSpeed);
    midOutTakeMotor.Set(0.2);
  } else {
    intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 0);
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
    intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, -0.2);
  }
}

/*void launcherAmp(bool enable) {
  if (enable) {
    setTopLauncher(-0.2, 0.2);
  }
}*/

//lifter function
void lifter(double rSideSpeed, double lSideSpeed) {
  // Right Lifter
  if (rSideSpeed > 0.05 || rSideSpeed < -0.05) {
    rLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, rSideSpeed);
  }
  else {
    rLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 0);
  }
  // Left Lifter
  if (lSideSpeed > 0.05 || lSideSpeed < -0.05) {
    lLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, lSideSpeed);
  }
  else {
    lLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 0);
  }
}

void backupDriveSystem(double forwardSpd, double backwardSpd, double dir){
  //Reduce turning sensitivity
  dir *= 0.4;
  // Forward and turning
  if (forwardSpd > 0 && (dir > 0.05 || dir < -0.05))
  {
    d_drive.ArcadeDrive(dir, forwardSpd * -1, false);
    // Backward and turning
  }
  else if (backwardSpd > 0 && (dir > 0.05 || dir < -0.05))
  {
    d_drive.ArcadeDrive(dir, backwardSpd, false);
    // Forward
  } else if (forwardSpd > 0) {
    d_drive.ArcadeDrive(0, forwardSpd * -1, false);
    // Backward
  }
  else if (backwardSpd > 0)
  {
    d_drive.ArcadeDrive(0, backwardSpd, false);
  }
  else if (dir > 0.05 || dir < -0.05)
  {
    d_drive.ArcadeDrive(dir, 0, false);
    // Stop
  }
  else
  {
    d_drive.ArcadeDrive(0, 0, false);
  }
}

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoDefaultLeave, kAutoDefaultLeave);
  m_chooser.AddOption(kAutoCustomSpeakerLeave, kAutoCustomSpeakerLeave);
  m_chooser.AddOption(kAutoCustomSpeaker, kAutoCustomSpeaker);
  m_chooser.AddOption(kAutoCustomDelayedSpeaker, kAutoCustomDelayedSpeaker);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  std::jthread visionThread(VisionThread);

  //Limit for drive train motors
  frMotor.SetSmartCurrentLimit(40);
  brMotor.SetSmartCurrentLimit(40);
  flMotor.SetSmartCurrentLimit(40);
  blMotor.SetSmartCurrentLimit(40);
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
  } else if (m_autoSelected == kAutoCustomSpeaker) {
    autoSpeaker();
  } else if (m_autoSelected == kAutoCustomDelayedSpeaker) {
    autoDelayedSpeaker();
  } else {
    autoLeave();
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoCustomSpeakerLeave) {
    autoSpeakerLeave();
  } else if (m_autoSelected == kAutoCustomSpeaker) {
    autoSpeaker();
  } else if (m_autoSelected == kAutoCustomDelayedSpeaker) {
    autoDelayedSpeaker();
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
  
  /* Right Lifter - Right Joystick*/double rJoyStick = manipulatorController.GetRightY();
  /* Left Lifter - Left Joystick*/double lJoyStick = manipulatorController.GetLeftY();

  backupDriveSystem(driveControllerRightTrigger, driveControllerLeftTrigger, driveControllerLeftJoyStickX);
  intake(lTrigger);
  outake(rTrigger);
  eject(lBumper);
  //launcherAmp(rBumper);
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
