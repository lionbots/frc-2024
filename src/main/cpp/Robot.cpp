// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/XboxController.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxAbsoluteEncoder.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <chrono>
#include <frc/filter/SlewRateLimiter.h>
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"
#include <frc/controller/PIDController.h>

// Used for auto (time based)
auto autoStartTime = std::chrono::high_resolution_clock::now();

// Motor controller for Drive system
rev::CANSparkMax frMotor{2, rev::CANSparkLowLevel::MotorType::kBrushless};
rev::CANSparkMax flMotor{1, rev::CANSparkLowLevel::MotorType::kBrushless};
rev::CANSparkMax brMotor{3, rev::CANSparkLowLevel::MotorType::kBrushless};
rev::CANSparkMax blMotor{4, rev::CANSparkLowLevel::MotorType::kBrushless};

// Motor controller groups (deprecated) please use leader and follower convention in a future commit.
// frc::MotorControllerGroup lMotorGroup(flMotor, blMotor);
// frc::MotorControllerGroup rMotorGroup(frMotor, brMotor);

// Differentialdrive Object
frc::DifferentialDrive d_drive{flMotor, frMotor};

// XBox Controller
frc::XboxController driveController(5);

// Motor Controller For Intake
ctre::phoenix::motorcontrol::can::TalonSRX intakeMotor{8};
rev::CANSparkMax upTopOutTakeMotor(7, rev::CANSparkLowLevel::MotorType::kBrushless);
rev::CANSparkMax bottomTopOutTakeMotor(6, rev::CANSparkLowLevel::MotorType::kBrushless);
rev::CANSparkMax midOutTakeMotor{5, rev::CANSparkLowLevel::MotorType::kBrushless};

// Motors Controller For Lifter
ctre::phoenix::motorcontrol::can::TalonSRX rLiftMotor{9};
ctre::phoenix::motorcontrol::can::TalonSRX lLiftMotor{10};

// XBox Controller
frc::XboxController manipulatorController(0);

// PID
frc::PIDController drivechainPID{0.017, 0.02, 0.0085}; 

double pidOutput;
bool liftersToggled = false;

void motorInit() {
  // reset the configuration parameters
  frMotor.RestoreFactoryDefaults();
  flMotor.RestoreFactoryDefaults();
  brMotor.RestoreFactoryDefaults();
  blMotor.RestoreFactoryDefaults();
  rLiftMotor.ConfigFactoryDefault();
  lLiftMotor.ConfigFactoryDefault();

  // activate break mode
  flMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  frMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  brMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  blMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  rLiftMotor.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
  lLiftMotor.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);

  // Limit for drive train motors
  frMotor.SetSmartCurrentLimit(40);
  brMotor.SetSmartCurrentLimit(40);
  flMotor.SetSmartCurrentLimit(40);
  blMotor.SetSmartCurrentLimit(40);
}
// function for running top launcher
// double launcherSpeed - specify percentage of launcher speed.
// bool sameDir - specify if you would like the launchers to move in the same direction.
void setTopLauncher(double launcherSpeed, bool sameDir)
{
  if (!sameDir)
  {
    upTopOutTakeMotor.Set(launcherSpeed);
    bottomTopOutTakeMotor.Set(launcherSpeed);
  }
  else
  {
    upTopOutTakeMotor.Set(launcherSpeed * -1);
    bottomTopOutTakeMotor.Set(launcherSpeed);
  }
}

// auto for getting leave points.
void autoLeave()
{
  if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(4000) + autoStartTime)
  {
    d_drive.ArcadeDrive(0, /* filter.Calculate(units::voltage::volt_t{-0.5}).value() */ -0.5, true);
  }
  else
  {
    d_drive.ArcadeDrive(0, 0, true);
  }
}

// Auto for shooting into the speaker then leaving.
void autoSpeakerLeave()
{
  if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(450) + autoStartTime)
  {
    d_drive.ArcadeDrive(0, 0.5, true);
  }
  else if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(1200) + autoStartTime)
  {
    setTopLauncher(1, false);
  }
  else if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(4100) + autoStartTime)
  {
    setTopLauncher(1, false);
    midOutTakeMotor.Set(0.2);
  }
  else if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(12100) + autoStartTime)
  {
    d_drive.ArcadeDrive(0, /* filter.Calculate(units::voltage::volt_t{0.5}).value() */ 0.5, true);
    setTopLauncher(0, false);
    midOutTakeMotor.Set(0);
  }
  else
  {
    d_drive.ArcadeDrive(0, 0, true);
  }
}

// Auto for shooting into the speaker.
void autoSpeaker()
{
  if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(1000) + autoStartTime)
  {
    setTopLauncher(1, false);
  }
  else if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(4000) + autoStartTime)
  {
    setTopLauncher(1, false);
    midOutTakeMotor.Set(0.2);
  }
  else
  {
    setTopLauncher(0, false);
    midOutTakeMotor.Set(0);
  }
}

// Auto for waiting, then shooting in the the speaker.
void autoDelayedSpeaker()
{
  if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(7000) + autoStartTime)
  {
    d_drive.ArcadeDrive(0, 0, false);
  }
  else if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(7450) + autoStartTime)
  {
    d_drive.ArcadeDrive(0, 0.5, false);
  }
  else if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(8350) + autoStartTime)
  {
    setTopLauncher(1, false);
  }
  else if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(11350) + autoStartTime)
  {
    setTopLauncher(1, false);
    midOutTakeMotor.Set(0.2);
  }
  else if (std::chrono::high_resolution_clock::now() < std::chrono::milliseconds(11700) + autoStartTime)
  {
    d_drive.ArcadeDrive(0, -0.5, false);
  }
  else
  {
    setTopLauncher(0, false);
    midOutTakeMotor.Set(0);
  }
}

// intake and outtake function
void intake(double intakeMotorSpeed)
{
  intakeMotorSpeed *= 2;
  // Intake
  if (intakeMotorSpeed > 1)
  {
    intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 1);
    midOutTakeMotor.Set(0.2);
  }
  else if (intakeMotorSpeed > 0)
  {
    intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, intakeMotorSpeed);
    midOutTakeMotor.Set(0.2);
  }
  else
  {
    intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 0);
  }
}

// intake and outtake function
void outake(double outTakeMotorSpeed, bool slowShot)
{
  // Outtake
  if (outTakeMotorSpeed > 0)
  {
    setTopLauncher(outTakeMotorSpeed, false);
    midOutTakeMotor.Set(0.2);
  }
  else if (slowShot)
  {
    setTopLauncher(0.7, false);
    midOutTakeMotor.Set(0.2);
  }
  else
  {
    setTopLauncher(0, false);
    midOutTakeMotor.Set(0);
  }
}

// Function for ejecting note back into intake.
void eject(bool ejectStatus)
{
  // Eject status
  if (ejectStatus)
  {
    setTopLauncher(-0.2, false);
    midOutTakeMotor.Set(-0.2);
    intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, -0.2);
  }
}

// lifter function
void lifter(bool liftUp, bool liftDown, bool toggleLifters, double lLifter, double rLifter)
{
  // Toggle Lifters between individual and synchronized
  if(toggleLifters)
  {
    liftersToggled = !liftersToggled;
  }

  // Lift Up
  if(!liftersToggled)
  {
    if (liftUp)
    {
      rLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, -1);
      lLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, -1);
    }
    // Lift Down
    else if (liftDown)
    {
      rLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 1);
      lLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 1);
    }
    else
    {
      rLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 0);
      lLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 0);
    }
  }
  else
  {
    if(lLifter > 0.05 || lLifter < -0.05)
    {
      lLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, lLifter);
    } else
    {
      lLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 0);
    }
    
    if (rLifter > 0.05 || rLifter < -0.05)
    {
      rLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, rLifter);
    } else
    {
      rLiftMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 0);
    }
  }
}

// Backup drive system to be replaced with field centric driving. Includes regular driving controls.
void backupDriveSystem(double forwardSpd, double backwardSpd, double dir, bool slowDown)
{

  // Reduce turning sensitivity
  dir *= 0.6;
  if (slowDown)
  {
    forwardSpd *= 0.3;
    backwardSpd *= 0.3;
    dir *= 0.5;
  }

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
  }
  else if (forwardSpd > 0)
  {
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

// Function for automatically aligning robot with detected note (target.)
void noteAlignment(int tvValue, double txValue, double throttle)
{
  // Set PID setpoint and tolerance
  drivechainPID.SetSetpoint(0);
  drivechainPID.SetTolerance(2, 3);
  // Calculate PID output
  pidOutput = drivechainPID.Calculate(txValue);
  frc::SmartDashboard::PutNumber("PID Output", pidOutput);
  // Check for target and if PID is at setpoint
  if (tvValue && !drivechainPID.AtSetpoint() && (txValue > 1.5 || txValue < -1.5))
  {
    // Rotate with PID output and move forward
    d_drive.ArcadeDrive(pidOutput * -1, throttle, true);
  }
  else
  {
    // Move forward
    d_drive.ArcadeDrive(0, throttle, true);
  }
}

void Robot::RobotInit()
{
  m_chooser.SetDefaultOption(kAutoDefaultLeave, kAutoDefaultLeave);
  m_chooser.AddOption(kAutoCustomSpeakerLeave, kAutoCustomSpeakerLeave);
  m_chooser.AddOption(kAutoCustomSpeaker, kAutoCustomSpeaker);
  m_chooser.AddOption(kAutoCustomDelayedSpeaker, kAutoCustomDelayedSpeaker);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  // Create thread for usb camera.
  std::jthread visionThread(VisionThread);

  drivechainPID.SetTolerance(2, 2);

  // run motor config
  motorInit();

  brMotor.Follow(frMotor);
  blMotor.Follow(flMotor);
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
void Robot::AutonomousInit()
{
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  autoStartTime = std::chrono::high_resolution_clock::now();
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoCustomSpeakerLeave)
  {
    autoSpeakerLeave();
  }
  else if (m_autoSelected == kAutoCustomSpeaker)
  {
    autoSpeaker();
  }
  else if (m_autoSelected == kAutoCustomDelayedSpeaker)
  {
    autoDelayedSpeaker();
  }
  else
  {
    autoLeave();
  }
}

void Robot::AutonomousPeriodic()
{
  if (m_autoSelected == kAutoCustomSpeakerLeave)
  {
    autoSpeakerLeave();
  }
  else if (m_autoSelected == kAutoCustomSpeaker)
  {
    autoSpeaker();
  }
  else if (m_autoSelected == kAutoCustomDelayedSpeaker)
  {
    autoDelayedSpeaker();
  }
  else
  {
    autoLeave();
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic()
{
  // Drive controller
  /* Backwards Throttle - Left Trigger */ double driveControllerLeftTrigger = driveController.GetLeftTriggerAxis();
  /* Forwards Throttle - Right Trigger */ double driveControllerRightTrigger = driveController.GetRightTriggerAxis();
  /* Turning - Left Joystick*/ double driveControllerLeftJoyStickX = driveController.GetLeftX();
  /* Slowdown -  Right Bumper */ double driveControllerRightBumper = driveController.GetRightBumper();
  /* Note Alignment - Y Button*/ bool driveControllerYButton = driveController.GetYButton();
  /* Y button first pressed - Y button*/ bool driveControllerYButtonPressed = driveController.GetYButtonPressed();

  // Manipulator controller
  /* Speaker Outtake - Right Trigger*/ double rTrigger = manipulatorController.GetRightTriggerAxis();
  /* Weaker Outtake - B Button */ bool bButton = manipulatorController.GetBButton();
  /* Intake  - Left Trigger*/ double lTrigger = manipulatorController.GetLeftTriggerAxis();
  /* Eject - Left Bumber*/ bool lBumper = manipulatorController.GetLeftBumper();

  /* Lifter Up - Y Button */ double yButton = manipulatorController.GetYButton();
  /* Lifter Down - A Button */ double aButton = manipulatorController.GetAButton();
  /* Left Lifter - Left Joystick */ double lJoystick = manipulatorController.GetLeftY();
  /* Right Lifter - Right Joystick */ double rJoystick = manipulatorController.GetRightY();
  /* Start Button - Lifter Toggle */ bool startButton = manipulatorController.GetStartButtonPressed();

  // Limelight Values, pulled from published network tables.
  int tv = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0.0);
  double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);

  if (driveControllerYButton)
  {
    noteAlignment(tv, tx, driveControllerLeftTrigger);
  }
  else
  {
    drivechainPID.Reset();
    backupDriveSystem(driveControllerRightTrigger, driveControllerLeftTrigger, driveControllerLeftJoyStickX, driveControllerRightBumper);
  }
  intake(lTrigger);
  outake(rTrigger, bButton);
  eject(lBumper);
  lifter(yButton, aButton, startButton, lJoystick, rJoystick);
}
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
