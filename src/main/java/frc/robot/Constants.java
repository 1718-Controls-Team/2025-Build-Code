// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;


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
public static final double kCoralRotateDeliveryPos = 6.19; //Was 6
public static final double kCoralRotateL4Pos = 6.84; //6.75
public static final double kCoralRotateHomePos = 3.29;  
public static final double kCoralRotateAlgaePos = 0.5;  
public static final double kCoralRotatePositionTolerance = 0.5;
public static final double kCoralUpPos = 0;
public static final double kCoralClimbPos = 5.929;

public static final double kCoralOutSpinSpeed = -10;
public static final double kCoralInSpinSpeed = 20;
public static final double kCoralHoldSpinSpeed = 10;
public static final double kCoralStopSpinSpeed = 0;

//180 max position if zero is all the way up
public static final double kTClimberUpPos = -56; //Was -69

// Elevator 0-33.5 is position range
public static final double kElevatorCoralIntakePos = 2.5;
public static final double kElevatorAlgaeIntakePos = 2.5;
public static final double kElevatorHomePos = 3;
public static final double kElevatorL2ScoringPos = 5.2; //Was 15
public static final double kElevatorL3ScoringPos = 18;
public static final double kElevatorL4ScoringPos = 39.9;
public static final double kElevatorL2AlgaePos = 16.8; //Was 20
public static final double kElevatorL3AlgaePos = 29.8; //Was 26
public static final double kElevatorClimbPos = 0;


// Algae is -0.441 to 3
public static final double kAlgaeOutSpinSpeed = -20;
public static final double kAlgaeInSpinSpeed = 30;
public static final double kAlgaeIdleSpinSpeed = 20;
public static final double kAlgaeStopSpinSpeed = 0;

public static final double kAlgaeIntakePos = 3.5;
public static final double kAlgaeClimbPos = 3.336;
public static final double kAlgaeHomePos = 1.1;
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

  public static final InvertedValue kCoralRotateDirection = InvertedValue.CounterClockwise_Positive;

  

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
  public static final double kCoralSpinSupplyCurrentLimit = 15;

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

  public static final InvertedValue kTClimberDirection = InvertedValue.Clockwise_Positive;


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
  public static final double kAprilX1Offset = 0; // 0.2??
  public static final double kAprilX2Offset = 0;
  public static final double kAprilX3Offset = 0;
  public static final double kAprilYOffset = 0;



  public static final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(2.0);
  public static final Distance kPositionTolerance = Centimeter.of(1.0);
  public static final LinearVelocity kSpeedTolerance = InchesPerSecond.of(1);
  public static final Time kEndTriggerDebounce = Seconds.of(0.1);
  public static final PathConstraints kPathContraints = new PathConstraints(2, 1.75, 1/2 * Math.PI, 1 * Math.PI);
  public static final Time kAutoAlignAdjustTimeout = Seconds.of(0.6);
  

  //Bunch of preset drive positions Xvalue, Yvalue, rotation
  public static final double[] kBlueTopLR = {3.57 + kAprilX2Offset, 5.15 - kAprilYOffset, 300}; //FIXED
  public static final double[] kBlueTopLL = {3.812 + kAprilX2Offset, 5.339 - kAprilYOffset, 300}; //FIXED
  public static final double[] kBlueTopRR = {5.003 + kAprilX2Offset, 5.432 - kAprilYOffset, 240}; //FIXED
  public static final double[] kBlueTopRL = {5.296 + kAprilX2Offset, 5.252 - kAprilYOffset, 240}; //FIXED
  public static final double[] kBlueRightR = {5.989 - kAprilX1Offset, 4.340, 180}; //FIXED
  public static final double[] kBlueRightL = {5.989 - kAprilX1Offset, 4.005, 180}; //FIXED
  public static final double[] kBlueBottomRR = {5.505 - kAprilX2Offset, 2.916, 120}; //FIXED
  public static final double[] kBlueBottomRL = {5.266 - kAprilX2Offset, 2.722, 120}; //FIXED
  public static final double[] kBlueBottomLR = {3.914 - kAprilX2Offset, 2.636 + kAprilYOffset, 60}; //FIXED
  public static final double[] kBlueBottomLL = {3.628 - kAprilX2Offset, 2.794 + kAprilYOffset, 60}; //FIXED
  public static final double[] kBlueLeftR = {3.03 + kAprilX1Offset, 3.763, 0}; //FIXED
  public static final double[] kBlueLeftL = {3.03 + kAprilX1Offset, 4.058, 0}; //FIXED
  public static final double[] kRedTopLR = {(17.55 - kBlueTopRL[0]), kBlueTopRL[1], 300}; //
  public static final double[] kRedTopLL = {(17.55 - kBlueTopRR[0]), kBlueTopRR[1], 300}; //
  public static final double[] kRedTopRR = {(17.55 - kBlueTopLL[0]), kBlueTopLL[1], 240}; //
  public static final double[] kRedTopRL = {(17.55 - kBlueTopLR[0]), kBlueTopLR[1], 240}; //
  public static final double[] kRedRightR = {(17.55 - kBlueLeftL[0]), kBlueLeftL[1], 180}; //
  public static final double[] kRedRightL = {(17.55 - kBlueLeftR[0]), kBlueLeftR[1], 180}; //
  public static final double[] kRedBottomRR = {(17.55 - kBlueBottomLL[0]), kBlueBottomLL[1], 120};
  public static final double[] kRedBottomRL = {(17.55 - kBlueBottomLR[0]), kBlueBottomLR[1], 120};
  public static final double[] kRedBottomLR = {(17.55 - kBlueBottomRL[0]), kBlueBottomRL[1], 60};
  public static final double[] kRedBottomLL = {(17.55 - kBlueBottomRR[0]), kBlueBottomRR[1], 60};
  public static final double[] kRedLeftR = {(17.55 - kBlueRightL[0]), kBlueRightL[1], 0};
  public static final double[] kRedLeftL = {(17.55 - kBlueRightR[0]), kBlueRightR[1], 0};
}
