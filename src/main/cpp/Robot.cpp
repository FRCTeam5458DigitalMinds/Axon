#include <string>
#include <Robot.h>
#include <sstream>
#include <WPILib.h>
#include <iostream>
#include <Encoder.h>
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


// Declarations

// PDP
frc::PowerDistributionPanel pdp{0};

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
// Helps Driving 
float signed_square(float x){
  return x * fabsf(x);
}
// Gyro
frc::ADXRS450_Gyro Gyro{}; 

// Cargo Intake
VictorSPX FrontLeftMid{4};
int Spiked = 0;

// Pneumatics
// Lift
frc::Solenoid CargoIntake{0};
bool CargoButton = false;
// HatchLock
frc::Solenoid HatchIntake{1};
bool HatchButton = false;

// Elevator Stuff
WPI_TalonSRX FrontRightBack{12};
WPI_VictorSPX FrontRightMid{11};
frc::SpeedControllerGroup Elevator{FrontRightBack, FrontRightMid};

// Joystick & Racewheel
frc::Joystick JoyAccel1{0}, Xbox{1}, RaceWheel{2};

// LimeLight
std::shared_ptr<NetworkTable> LimeTable = NetworkTable::GetTable("limelight");

// Variables for intake
bool threeFirstPressed = false;
float intakeCurrentStart, intakeCurrentEnd;
int intakeCurrentCounter = 0;
int intakeCurrentFrames = 3;
int intakeCurrentThreshold = 10;
bool intakeStalled = false;

// Straightens out the bot
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
void Robot::TeleopInit() {

}

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

  // Intakes the ball when button 3 is pressed
  if (Xbox.GetRawButton(3))
  {
    //Check if the intakeStalled variable is false, meaning that the intake motor is not currently stalling
    if (!intakeStalled)
    {

      //If the motor is not currently stalled, a counter intakeCurrentCounter is started. This counter is 3 frames long. 
      if (intakeCurrentCounter == 0) {
      
        //At the start of the counter, store the current electrical current into a variable named intakeCurrentStart
        //Also raise the counter by 1 so that this section of code only runs once and the counter is initialized.
        intakeCurrentStart = pdp.GetCurrent(4);
        intakeCurrentCounter = intakeCurrentCounter + 1;

      }
      //This next line artifically creates a delay of 3 frames, increasing the counter value by 1 with every passing frame
      else if (intakeCurrentCounter < intakeCurrentFrames) intakeCurrentCounter = intakeCurrentCounter + 1;
      //Once the counter reaches 3, three frames have passed and its now time to check the current electrical current again
      else
      {

        //After the 3 frame delay, we check for the electrical current again and store this second value in a variable named
        //intakeCurrentEnd
        //We also reset the counter back to 0 so that this continues to work as long as button 3 is being pressed
        intakeCurrentEnd = pdp.GetCurrent(4);
        intakeCurrentCounter = 0;
        
        //Now we check for the difference between the electrical current at the start of the 3 frames and at the end of the 3 frames
        //If the difference between the two is very low, in this case less than 2, then we know that the intake is either stalling
        //or that the intake is currently not intaking a ball at all.
        //The next argument in the if statement checks if the electrical current at the end of the 3 frames is greater than the
        //threshold, currently set to 10.
        //This argument lets us know that the motor is probably stalling, to differentiate it from just not intaking anything
        if ((abs(intakeCurrentEnd - intakeCurrentStart) < 2) && intakeCurrentEnd > intakeCurrentThreshold)
        {

          //If both of the above arguments are true, we set the intake motor to zero because it must be stalling
          //We also set intakeStalled variable to true so that the whole system does not start over until button 3 is released
          //and pressed again
          FrontLeftMid.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
          intakeStalled = true;

        }
        //If either one of the above arguements are false, it must not be stalling because of an intaked ball so it continues to spin
        //the motor
        else FrontLeftMid.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.5);

      }

    }

  }
  //When button 3 is not pressed, we set the intake motor power to zero.
  //Since the motor cannot be stalling if it's not even running, we also set the intakeStalled variable to false
  else
  { 
  
    //Spit the ball if button 1 is pressed when button 3 is not being pressed
    if (Xbox.GetRawButton(1)) FrontLeftMid.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);
    else 
    {
      
      FrontLeftMid.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      intakeStalled = false;

    }

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