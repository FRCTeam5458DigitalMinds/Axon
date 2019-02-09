#include <Robot.h>
#include <WPILib.h>
#include <iostream>
#include <frc/Timer.h>
#include <TimedRobot.h>
#include <frc/Solenoid.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/SpeedControllerGroup.h>
#include <NetworkTables/NetworkTable.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SmartDashboard.h>

//Declarations

// Right Side Motors
WPI_TalonSRX BackRightBack{15};
WPI_VictorSPX BackRightMid{14};
WPI_VictorSPX BackRightFront{13};
// Left Side Motors
WPI_TalonSRX BackLeftBack{0};
WPI_VictorSPX BackLeftMid{1};
WPI_VictorSPX BackLeftFront{2};
// Speed Controller Groups
frc::SpeedControllerGroup RightMotors{BackRightFront,BackRightMid,BackRightBack};
frc::SpeedControllerGroup LeftMotors{BackLeftFront, BackLeftMid, BackLeftBack};
// Drive Train
frc::DifferentialDrive DriveTrain{LeftMotors, RightMotors};

float signed_square(float x){
  return x * fabsf(x);
}

// Gyro
frc::ADXRS450_Gyro Gyro{}; 

//Cargo Intake
VictorSPX FrontLeftMid{4};

// Pneumatics/ Lift
frc::Solenoid CargoIntake{0};
bool CargoButton = false;
// HatchLock
frc::Solenoid HatchIntake{1};
bool HatchButton = false;

// Elevator Stuff
TalonSRX FrontRightBack{12};
VictorSPX FrontRightMid{11};

// Joystick & Racewheel
frc::Joystick JoyAccel1{0}, Xbox{1}, RaceWheel{2};

// LimeLight
std::shared_ptr<NetworkTable> LimeTable = NetworkTable::GetTable("limelight");

//Straightens out the bot
float LastSumAngle;

/*Called on robot connection*/
void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  RightMotors.SetInverted(true);
  LeftMotors.SetInverted(false);

  HatchIntake.Set(false);
  CargoIntake.Set(false);

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
  //Gets axis for each controller
  double JoyY = JoyAccel1.GetY();
  double WheelX = RaceWheel.GetX();

  double SquaredWheelInput = signed_square(WheelX);

  //Power get's cut from one side of the bot to straighten out when driving straight
  float sumAngle = Gyro.GetAngle();
  float derivAngle = sumAngle - LastSumAngle;
  float correctionAngle = (sumAngle *.1) + (derivAngle *.2);

  // Intake Lift
  if (Xbox.GetRawButton(5)){
    if (!CargoButton){
      CargoIntake.Set(!CargoIntake.Get());
      CargoButton = true;
    }
  } else {
    CargoButton = false;
    HatchButton = false;
  }

  // Intakes the ball
  if (Xbox.GetRawButton(3)){
    FrontLeftMid.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -.5);
  // Spits the ball
  } else if (Xbox.GetRawButton(1)){
    FrontLeftMid.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);
  } else {
    FrontLeftMid.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
  }

  if (RaceWheel.GetRawButton(5)){
    RightMotors.Set(SquaredWheelInput);
    LeftMotors.Set(-SquaredWheelInput);
  } else if(JoyY > 0.02 || JoyY < -0.02){
    DriveTrain.ArcadeDrive(-SquaredWheelInput, JoyY, true);
  } else {
    RightMotors.Set(0);
    LeftMotors.Set(0);
  }
  


  
  //Straightens out bot here when driving straight
  LastSumAngle = sumAngle;

}

/*Called every robot packet in testing mode*/
void Robot::TestPeriodic() {}


/*Starts the bot*/
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif