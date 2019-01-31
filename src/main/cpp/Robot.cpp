/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <Robot.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <frc/Timer.h>
#include <frc/drive/DifferentialDrive.h>
#include <TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/ADXRS450_Gyro.h>


// Right Side Motors
TalonSRX BackRightBack{15};
VictorSPX BackRightmid{14};
VictorSPX BackRightFront{13};
// Left Side Motors
TalonSRX BackLeftBack{0};
VictorSPX BackLeftmid{1};
VictorSPX BackLeftFront{2};

// Joystick & Racewheel
frc::Joystick JoyAccel1{0}, RaceWheel{2};

// Gyro
frc::ADXRS450_Gyro Gyro{}; 
float lastSumAngle;

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  
  Gyro.Reset();

  // Right Side Motors
  /*BackRightBack.Set(ControlMode::PercentOutput, .2);
  BackRightmid.Set(ControlMode::PercentOutput, .2);
  BackRightFront.Set(ControlMode::PercentOutput, .2);
  // Left Side Motors
  BackLeftBack.Set(ControlMode::PercentOutput, -.1);
  BackLeftmid.Set(ControlMode::PercentOutput, -.1);
  BackLeftFront.Set(ControlMode::PercentOutput, -.1);
  */
}

/**
 * This function is called every robot packet, no matter the mode. Use
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
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

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

void RightSpeedPercentage(float percentage){
  BackRightBack.Set(ControlMode::PercentOutput, percentage);
  BackRightmid.Set(ControlMode::PercentOutput, percentage);
  BackRightFront.Set(ControlMode::PercentOutput, percentage);
}

void LeftSpeedPercentage(float percentage){
  BackLeftBack.Set(ControlMode::PercentOutput, percentage);
  BackLeftmid.Set(ControlMode::PercentOutput, percentage);
  BackLeftFront.Set(ControlMode::PercentOutput, percentage);
}

void Robot::TeleopPeriodic() {
  double yInput = JoyAccel1.GetY();
  double xInput = RaceWheel.GetX();

  float sumAngle = Gyro.GetAngle();
  float derivAngle = sumAngle - lastSumAngle;
  float correctionAngle = (sumAngle * 0.1) + (derivAngle * .2);

  // Point turning (BUTTON = 5)
  if(RaceWheel.GetRawButton(5)){
    RightSpeedPercentage(xInput);
    LeftSpeedPercentage(xInput);  
  }
  // Drive Forward and Turn
  else if((yInput > 0.06 || yInput < -0.06) && (xInput > 0.01 || xInput < -0.01)){
    RightSpeedPercentage(-(yInput + 0.5*xInput));
    std::cout << yInput + 0.5*xInput << std::endl;
    LeftSpeedPercentage((yInput + 0.5*xInput)); 
  }
  // Drive Forward and no Turn
  else if(yInput > 0.06 || yInput < -0.06){
    RightSpeedPercentage(-yInput - correctionAngle);
    LeftSpeedPercentage(yInput - correctionAngle); 
  }
  // Turn off motors if button (5) not pressed 
  else 
  {
    RightSpeedPercentage(0);
    LeftSpeedPercentage(0);
  }

  lastSumAngle = sumAngle;  

}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif