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

//Motor Controller For Intake
rev::CANSparkMax intakeMoter(1, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
rev::CANSparkMax topOutTakeMotor(2, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
rev::CANSparkMax midOutTakeMotor{3, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

//Motors Controller For Lifter
ctre::phoenix::motorcontrol::can::TalonSRX rLiftMotor{1};
ctre::phoenix::motorcontrol::can::TalonSRX lLiftMotor{1};

//XBox Controller
frc::XboxController manipulatorController(0);

//intake and outtake
void launcher (double topOutTakeMotorSpeed, double midOutTakeMotorSpeed) {
  if (topOutTakeMotorSpeed > 0) {
    intakeMoter.Set(topOutTakeMotorSpeed);
  } 
  else if (midOutTakeMotorSpeed > 0) {
    topOutTakeMotor.Set(midOutTakeMotorSpeed);
    midOutTakeMotor.Set(0.2);
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
  double rTrigger = manipulatorController.GetRightTriggerAxis();
  double lTrigger = manipulatorController.GetLeftTriggerAxis();
  
  double rJoyStick = manipulatorController.GetRightY() * -1;
  double lJoyStick = manipulatorController.GetLeftY() * -1;
  
  double topOutTakeMotorSpeed = rTrigger;
  double midOutTakeMotorSpeed = lTrigger;

  launcher(topOutTakeMotorSpeed, midOutTakeMotorSpeed);

  // Right Lifter
  if (rJoyStick > 0.05) {
    rLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, rJoyStick);
  }
  else if (rJoyStick < -0.05) {
    rLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, rJoyStick * -1);
  }
  else {
    rLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 0);
  }

  // Left Lifter
  if (lJoyStick > 0.05) {
    lLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, lJoyStick);
  }
  else if (lJoyStick < -0.05) {
    lLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, lJoyStick * -1);
  }
  else {
    lLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 0);
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
