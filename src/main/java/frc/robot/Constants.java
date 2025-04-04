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
public static final double kCoralRotateDeliveryPos = 8.25; //Was 6.19
public static final double kCoralRotateL4Pos = 8.73; //6.75
public static final double kCoralAutonL4Pos = 8.28; //6.75
public static final double kCoralRotateHomePos = 4.2;   
public static final double kCoralRotateAlgaePos = 0.5;  
public static final double kCoralRotatePositionTolerance = 0.5;
public static final double kCoralUpPos = 0;
public static final double kCoralClimbPos = 5.929;

public static final double kCoralOutSpinSpeed = -28;
public static final double kCoralInSpinSpeed = 35;
public static final double kCoralHoldSpinSpeed = 5;
public static final double kCoralStopSpinSpeed = 0;

//180 max position if zero is all the way up
public static final double kTClimberUpPos = -40; //Was -69

// Elevator 0-33.5 is position range
public static final double kElevatorCoralIntakePos = 0;
public static final double kElevatorAlgaeIntakePos = 2.5;
public static final double kElevatorHomePos = 3;
public static final double kElevatorL2ScoringPos = 5.2; //Was 15
public static final double kElevatorL3ScoringPos = 19.1;
public static final double kElevatorL4ScoringPos = 39.9;
public static final double kElevatorL2AlgaePos = 16.8; //Was 20
public static final double kElevatorL3AlgaePos = 29.8; //Was 26
public static final double kElevatorClimbPos = 0;


// Algae is -0.441 to 3
public static final double kAlgaeOutSpinSpeed = -20;
public static final double kAlgaeInSpinSpeed = 30;
public static final double kAlgaeIdleSpinSpeed = 10;
public static final double kAlgaeStopSpinSpeed = 0;

public static final double kAlgaeIntakePos = 3.5;
public static final double kAlgaeClimbPos = 3.336;
public static final double kAlgaeNetPos = 5.7;
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
  public static final double kCoralSpinSupplyCurrentLimit = 20;

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
  
  public static final double kAlgaeIntake1SupplyCurrentLimit = 20;
  public static final double kAlgaeIntake1VoltageClosedLoopRampPeriod = 0;

  public static final InvertedValue kAlgaeIntake1Direction = InvertedValue.Clockwise_Positive;

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

  public static final double kAlgaeIntake2SupplyCurrentLimit = 20;
  public static final double kAlgaeIntake2VoltageClosedLoopRampPeriod = 0;

  public static final InvertedValue kAlgaeIntake2Direction = InvertedValue.CounterClockwise_Positive;

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
  public static final Time kAutoAlignAdjustTimeout = Seconds.of(1.0);
  

  //Bunch of preset drive positions Xvalue, Yvalue, rotation
  public static final double[] kBlueTopLR = {3.379 + kAprilX2Offset, 5.148 - kAprilYOffset, 300};     //FIXED  
  public static final double[] kBlueTopLL = {3.725 + kAprilX2Offset, 5.3603 - kAprilYOffset, 300};    //FIXED
  public static final double[] kBlueTopRR = {5.595 + kAprilX2Offset, 5.148 - kAprilYOffset, 240};     //FIXED
  public static final double[] kBlueTopRL = {5.249 + kAprilX2Offset, 5.3603 - kAprilYOffset, 240};    //FIXED
  public static final double[] kBlueRightR = {5.989 - kAprilX1Offset, 4.340, 180}; 
  public static final double[] kBlueRightL = {6.011 - kAprilX1Offset, 4.04, 180}; //Maybe
  public static final double[] kBlueBottomRR = {5.595 - kAprilX2Offset, 3.379, 120};                  //FIXED
  public static final double[] kBlueBottomRL = {5.249 - kAprilX2Offset, 2.7197, 120};                 //FIXED
  public static final double[] kBlueBottomLR = {3.379 - kAprilX2Offset, 3.379 + kAprilYOffset, 60};   //FIXED
  public static final double[] kBlueBottomLL = {3.725 - kAprilX2Offset, 2.7197 + kAprilYOffset, 60};  //FIXED
  public static final double[] kBlueLeftR = {3.03 + kAprilX1Offset, 3.763, 0}; 
  public static final double[] kBlueLeftL = {2.963 + kAprilX1Offset, 4.04, 0}; //Maybe
  public static final double[] kRedTopLR = {(12.104), 5.148, 120};                              // Right fixed
  public static final double[] kRedTopLL = {(11.539), 5.3603, 120};                             //FIXED
  public static final double[] kRedTopRR = {(14.022), 5.148, 60};                               //Right Fixed
  public static final double[] kRedTopRL = {(13.825), 5.3603, 60};                              //FIXED
  public static final double[] kRedRightR = {(17.55 - kBlueLeftL[0]), kBlueLeftL[1], 0}; 
  public static final double[] kRedRightL = {(14.587), 4.04, 0};                                //FIXED
  public static final double[] kRedBottomRR = {(14.022), 2.932, 300};                           //RIGHT FIXED
  public static final double[] kRedBottomRL = {(13.825), 2.7197, 300};                          //THIS IS THE OG FIXED 
  public static final double[] kRedBottomLR = {(12.104), 2.932, 240};                           //RIGHT FIXED
  public static final double[] kRedBottomLL = {(14.587), 2.7197, 240};                          //FIXED
  public static final double[] kRedLeftR = {(17.55 - kBlueRightL[0]), kBlueRightL[1], 180};
  public static final double[] kRedLeftL = {(11.539), 4.04, 180};                               //FIXED
}
/*
 * Math
 * Center of Red Reef:
 * x = 13.063
 * y = 4.04
 * 
 * For scoring on right Branch
 * (14.022-13.063)^2 + (4.04-2.932)^2
 * r = 2.147345
 * Standard X change = 0.959
 * Standard Y change = 1.108
 * 
 * 
 * For scoring on left Branch
 * Standard X change = 0.762
 * Standard Y change = 1.3203
 * (13.825-13.063)^2 + (4.04-2.7197)^2
 * r = 1.5244133593 for all red(?) left scores
 */