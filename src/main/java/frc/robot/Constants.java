// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;


public final class Constants {
//Cancoder constants
public static final int kElevator1CanID = 13;
public static final int kElevator2CanID = 14;
public static final int kCoralRotateCanID = 15;
public static final int kCoralRotateCancoderCanID = 16;
public static final int kCoralIntakeCanID = 17;
public static final int kAlgaeIntakeRotateCanID = 18;
public static final int kAlgaeIntakeRotateCancoderCanID = 19;
public static final int kAlgaeIntakeSpin1CanID = 20;
public static final int kAlgaeIntakeSpin2CanID = 21;

public static final String kLimelightName = "limelight-lime";

/*
 * ####################################################################################################################################
 * ##################################################### Positions ###################################################################
 * ####################################################################################################################################
*/

// Coral rotate position range is 0-11
public static final double kCoralRotateDeliveryPos = 6; //Was 5
public static final double kCoralRotateHomePos = 3;  
public static final double kCoralRotateAlgaePos = 1.5;  
public static final double kCoralRotatePositionTolerance = 0.5;
public static final double kCoralUpPos = 0;
public static final double kCoralClimbPos = 6.929;

public static final double kCoralOutSpinSpeed = -10;
public static final double kCoralInSpinSpeed = 10;
public static final double kCoralHoldSpinSpeed = 3;
public static final double kCoralStopSpinSpeed = 0;

//180 max position if zero is all the way up
public static final double kTClimberUpPos = 38;

// Elevator 0-33.5 is position range
public static final double kElevatorCoralIntakePos = 7.5;
public static final double kElevatorAlgaeIntakePos = 4.5;
public static final double kElevatorHomePos = 3;
public static final double kElevatorL2ScoringPos = 14; //Was 15
public static final double kElevatorL3ScoringPos = 25;
public static final double kElevatorL4ScoringPos = 33;
public static final double kElevatorL2AlgaePos = 22;
public static final double kElevatorL3AlgaePos = 31;
public static final double kElevatorClimbPos = 0;



// Algae is -0.441 to 3
public static final double kAlgaeOutSpinSpeed = -20;
public static final double kAlgaeInSpinSpeed = 20;
public static final double kAlgaeIdleSpinSpeed = 3;
public static final double kAlgaeStopSpinSpeed = 0;

public static final double kAlgaeIntakePos = 3.5;
public static final double kAlgaeClimbPos = 3.336;
public static final double kAlgaeHomePos = 1.37;
public static final double kAlgaeIntakePositionTolerance = 0.05;



/*
 * ####################################################################################################################################
 * ####################################################### Coral Rotate ###############################################################
 * ####################################################################################################################################
*/
  public static final double kCoralRotateProportional = 1; 
  public static final double kCoralRotateIntegral = 0;
  public static final double kCoralRotateDerivative = 0;

  public static final double kCoralRotateVelocityFeedForward = 0;
  public static final double kCoralRotateGravityFeedForward = 0;

  public static final double kCoralMaxReverseVoltage = -11;
  public static final double kCoralMaxForwardVoltage = 11;
  public static final double kCoralRotateVoltageClosedLoopRampPeriod = 0;
  public static final double kCoralRotateSupplyCurrentLimit = 40;

  public static final double kCoralRotateMotionMagicAcceleration = 100;
  public static final double kCoralRotateMotionMagicCruiseVelocity = 50;

  public static final InvertedValue kCoralRotateDirection = InvertedValue.Clockwise_Positive;

  

/*
 * ####################################################################################################################################
 * ##################################################### Coral Spin ###################################################################
 * ####################################################################################################################################
*/
  public static final double kCoralSpinDerivative = 0;
  public static final double kCoralSpinIntegral = 0;
  public static final double kCoralSpinProportional = 0.07;  

  public static final double kCoralSpinVelocityFeedForward = 0.122;
  public static final double kCoralSpinGravityFeedForward = 0;
  public static final double kCoralSpinStaticFeedForward = 0;

  public static final double kCoralSpinMaxForwardVoltage = 11;
  public static final double kCoralSpinMaxReverseVoltage = -11;

  public static final InvertedValue kCoralSpinDirection = InvertedValue.Clockwise_Positive;
  public static final double kCoralSpinSupplyCurrentLimit = 40;

  public static final double kCoralSpinVoltageClosedLoopRampPeriod = 0;

/*
 * ####################################################################################################################################
 * ###################################################### Climber #####################################################################
 * ####################################################################################################################################
*/
  public static final double kTClimberProportional = 0.2; 
  public static final double kTClimberIntegral = 0;
  public static final double kTClimberDerivative = 0;

  public static final double kTClimberVelocityFeedForward = 0;
  public static final double kTClimberGravityFeedForward = 0;
  public static final double kTClimberStaticFeedForward = 0;

  public static final double kTClimberVoltageClosedLoopRampPeriod = 0;
  public static final double kTClimberMaxReverseVoltage = -11;
  public static final double kTClimberMaxForwardVoltage = 11;
  public static final double kTClimberSupplyCurrentLimit = 40;

  public static final InvertedValue kTClimberDirection = InvertedValue.CounterClockwise_Positive;


/*
 * ####################################################################################################################################
 * ################################################## Elevator 1 #####################################################################
 * ####################################################################################################################################
*/
  public static final double kElevator1Proportional = 2.5;
  public static final double kElevator1Integral = 0;
  public static final double kElevator1Derivative = 0.2;

  public static final double kElevator1VelocityFeedForward = 0;
  public static final double kElevator1GravityFeedForward = 0.2;
  public static final double kElevator1StaticFeedForward = 0;

  public static final InvertedValue kElevator1Direction = InvertedValue.Clockwise_Positive;

  public static final double kElevator1SupplyCurrentLimit = 40;
  public static final double kElevator1VoltageClosedLoopRampPeriod = 0;
  public static final double kElevator1MaxForwardVoltage = 11;
  public static final double kElevator1MaxReverseVoltage = -11;

/*
 * ####################################################################################################################################
 * ################################################## Elevator 2 #####################################################################
 * ####################################################################################################################################
*/

  public static final double kElevator2Proportional = 2.5;
  public static final double kElevator2Integral = 0;
  public static final double kElevator2Derivative = 0.2;

  public static final double kElevator2VelocityFeedForward = 0;
  public static final double kElevator2GravityFeedForward = 0.2;
  public static final double kElevator2StaticFeedForward = 0;

  public static final InvertedValue kElevator2Direction = InvertedValue.CounterClockwise_Positive;

  public static final double kElevator2SupplyCurrentLimit = 40;
  public static final double kElevator2VoltageClosedLoopRampPeriod = 0;
  public static final double kElevator2MaxForwardVoltage = 11;
  public static final double kElevator2MaxReverseVoltage = -11;



/*
 * ####################################################################################################################################
 * ################################################## ALGAE INTAKE SPIN ###############################################################
 * ####################################################################################################################################
*/
  public static final double kAlgaeIntake1Proportional = 0.07;
  public static final double kAlgaeIntake1Integral = 0;
  public static final double kAlgaeIntake1Derivative = 0;
  
  public static final double kAlgaeIntake1VelocityFeedForward = 0.122;
  public static final double kAlgaeIntake1GravityFeedForward = 0;
  public static final double kAlgaeIntake1StaticFeedForward = 0;

  public static final double kAlgaeIntake1MaxForwardVoltage = 11;
  public static final double kAlgaeIntake1MaxReverseVoltage = -11;
  
  public static final double kAlgaeIntake1SupplyCurrentLimit = 40;
  public static final double kAlgaeIntake1VoltageClosedLoopRampPeriod = 0;

  public static final InvertedValue kAlgaeIntake1Direction = InvertedValue.CounterClockwise_Positive;

/*
 * ####################################################################################################################################
 * ################################################# ALGAE INTAKE SPIN 2 ##############################################################
 * ####################################################################################################################################
*/

  public static final double kAlgaeIntake2Proportional = 0.07;
  public static final double kAlgaeIntake2Integral = 0;  
  public static final double kAlgaeIntake2Derivative = 0;

  public static final double kAlgaeIntake2VelocityFeedForward = 0.122;
  public static final double kAlgaeIntake2GravityFeedForward = 0;
  public static final double kAlgaeIntake2StaticFeedForward = 0;

  public static final double kAlgaeIntake2MaxForwardVoltage = 11;
  public static final double kAlgaeIntake2MaxReverseVoltage = -11;

  public static final double kAlgaeIntake2SupplyCurrentLimit = 40;
  public static final double kAlgaeIntake2VoltageClosedLoopRampPeriod = 0;

  public static final InvertedValue kAlgaeIntake2Direction = InvertedValue.Clockwise_Positive;

/*
 * ####################################################################################################################################
 * ################################################# ALGAE INTAKE ROTATE ##############################################################
 * ####################################################################################################################################
*/

  public static final double kAlgaeIntakeRotateProportional = 7; 
  public static final double kAlgaeIntakeRotateIntegral = 0;
  public static final double kAlgaeIntakeRotateDerivative = 0.2;

  public static final double kAlgaeIntakeRotateVelocityFeedForward = 0.2;
  public static final double kAlgaeIntakeRotateGravityFeedForward = 0;
  public static final double kAlgaeIntakeRotateStaticFeedForward = 0;

  public static final double kAlgaeIntakeRotateMaxReverseVoltage = -11;
  public static final double kAlgaeIntakeRotateMaxForwardVoltage = 11;

  public static final double kAlgaeIntakeRotateVoltageClosedLoopRampPeriod = 0;
  public static final double kAlgaeIntakeRotateSupplyCurrentLimit = 40;

  public static final InvertedValue kAlgaeIntakeRotateDirection = InvertedValue.CounterClockwise_Positive;



/*
 * ####################################################################################################################################
 * ##################################################### THE OTHER STUFF ##############################################################
 * ####################################################################################################################################
*/

  //Custom Brownout voltage for the RIO2.
  public static final double kCustomBrownout = 6.0;


  //Bunch of preset drive positions Xvalue, Yvalue, rotation
  public static final double[] kBlueTopLR = {3.722, 5.019, -60};
  public static final double[] kBlueTopLL = {4.017, 5.187, -60};
  public static final double[] kBlueTopRR = {4.969, 5.191, -120};
  public static final double[] kBlueTopRL = {5.256, 5.025, -120};
  public static final double[] kBlueRightR = {5.739, 4.193, 180};
  public static final double[] kBlueRightL = {5.739, 3.860, 180};
  public static final double[] kBlueBottomRR = {5.257, 3.027, 120};
  public static final double[] kBlueBottomRL = {4.966, 2.866, 120};
  public static final double[] kBlueBottomLR = {4.009, 2.863, 60};
  public static final double[] kBlueBottomLL = {3.716, 3.031, 60};
  public static final double[] kBlueLeftR = {3.236, 3.862, 0};
  public static final double[] kBlueLeftL = {3.236, 4.192, 0};
  public static final double[] kRedTopLR = {(17.55 - kBlueTopRL[0]), kBlueTopRL[1], -60};
  public static final double[] kRedTopLL = {(17.55 - kBlueTopRR[0]), kBlueTopRR[1], -60};
  public static final double[] kRedTopRR = {(17.55 - kBlueTopLL[0]), kBlueTopLL[1], -120};
  public static final double[] kRedTopRL = {(17.55 - kBlueTopLR[0]), kBlueTopLR[1], -120};
  public static final double[] kRedRightR = {(17.55 - kBlueLeftL[0]), kBlueLeftL[1], -180};
  public static final double[] kRedRightL = {(17.55 - kBlueLeftR[0]), kBlueLeftR[1], -180};
  public static final double[] kRedBottomRR = {(17.55 - kBlueBottomLL[0]), kBlueBottomLL[1], 120};
  public static final double[] kRedBottomRL = {(17.55 - kBlueBottomLR[0]), kBlueBottomLR[1], 120};
  public static final double[] kRedBottomLR = {(17.55 - kBlueBottomRL[0]), kBlueBottomRL[1], 60};
  public static final double[] kRedBottomLL = {(17.55 - kBlueBottomRR[0]), kBlueBottomRR[1], 60};
  public static final double[] kRedLeftR = {(17.55 - kBlueRightL[0]), kBlueRightL[1], 0};
  public static final double[] kRedLeftL = {(17.55 - kBlueRightR[0]), kBlueRightR[1], 0};
}
