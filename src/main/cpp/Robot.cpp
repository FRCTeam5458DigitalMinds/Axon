/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <Robot.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include <frc/drive/DifferentialDrive.h>
#include <TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/ADXRS450_Gyro.h>
#include <WPILib.h>
#include <ctre/Phoenix.h>


// Right Side Motors
WPI_TalonSRX BackRightBack{15};
WPI_VictorSPX BackRightmid{14};
WPI_VictorSPX BackRightFront{13};
// Left Side Motors
WPI_TalonSRX BackLeftBack{0};
WPI_VictorSPX BackLeftmid{1};
WPI_VictorSPX BackLeftFront{2};

// Joystick & Racewheel
frc::Joystick JoyAccel1{0}, RaceWheel{2};

// Gyro
frc::ADXRS450_Gyro Gyro{}; 

/* -+- Drive Functions -+- */
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

/*Called on robot connection*/
void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  
  Gyro.Reset();
}

/*Called on every robot packet, no matter what mode*/
void Robot::RobotPeriodic() {}

/*Called when Autonomous is enabled*/
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  std::cout << "Auto selected: " << m_autoSelected << std::endl;
  if (m_autoSelected == kAutoNameCustom) {
  } else {
  }
}

/*Called every robot packet in auto*/
void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
  } else {
  }
}

/*Called when teleop is enabled*/
void Robot::TeleopInit() {}

/*Called every robot packet in teleop*/
void Robot::TeleopPeriodic() {
  double yInput = JoyAccel1.GetY();
  double xInput = RaceWheel.GetX();

  float turnFact = 0.5;
  float yInputBuffer = 0.06;
  float xInputBuffer = 0.01;

  // Point turning (BUTTON = 5)
  if (RaceWheel.GetRawButton(5)){
    RightSpeedPercentage(xInput);
    LeftSpeedPercentage(xInput);  
  }

  // Move turning
  else if ((yInput > yInputBuffer || yInput < -yInputBuffer) && (xInput > xInputBuffer || xInput < -xInputBuffer)){
    // If turning right
    if (xInput > xInputBuffer){
      // Drive left at yInput percentage (100% Joystick 0% Wheel)
      LeftSpeedPercentage(yInput);
      // Drive right at yInput - turnFact% of xInput (100% Joystick, turnFact% Wheel) (At 100% on both Joystick and Wheel, the right wheels will turn at turnFact the speed of the left wheels)
      RightSpeedPercentage(yInput - (turnFact*xInput));
    }
    // If turning left
    else if (xInput < xInputBuffer){
      /// Drive right at yInput percentage (100% Joystick 0% Wheel)
      RightSpeedPercentage(yInput);
      // Drive left at yInput - turnFact of xInput (100% Joystick, turnFact% Wheel) (At 100% on both Joystick and Wheel, the right wheels will turn at turnFact the speed of the left wheels)
      LeftSpeedPercentage(yInput - (turnFact*xInput));       
    }
  }

  // Move forward
  else if (yInput > yInputBuffer || yInput < -yInputBuffer){
    RightSpeedPercentage(-yInput);
    LeftSpeedPercentage(yInput); 
  }

  // Turn off motors nothing above is happening 
  else 
  {
    RightSpeedPercentage(0);
    LeftSpeedPercentage(0);
  }
}

/*Called every robot packet in testing mode*/
void Robot::TestPeriodic() {}

/*Starts the bot*/
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif