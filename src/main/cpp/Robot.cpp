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
#include <frc/SpeedControllerGroup.h>
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

frc::SpeedControllerGroup RightMotors{BackRightFront,BackRightmid,BackRightBack};
frc::SpeedControllerGroup LeftMotors{BackLeftFront, BackLeftmid, BackLeftBack};

frc::DifferentialDrive DriveTrain{LeftMotors, RightMotors};

// Joystick & Racewheel
frc::Joystick JoyAccel1{0}, RaceWheel{2};

// Gyro
frc::ADXRS450_Gyro Gyro{}; 

/*Called on robot connection*/
void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  RightMotors.SetInverted(true);
  LeftMotors.SetInverted(false);
  
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

  DriveTrain.ArcadeDrive(xInput, yInput);

  // Point turning 
  if (RaceWheel.GetRawButton(5)){
    RightMotors.Set(xInput);
    LeftMotors.Set(xInput);  
  }

}

/*Called every robot packet in testing mode*/
void Robot::TestPeriodic() {}

/*Starts the bot*/
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif