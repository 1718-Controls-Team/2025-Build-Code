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
public static final double kCoralRotateHomePos = 3.5; //4.2  
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
public static final double kElevatorL2AlgaePos = 16.32; //Was 20
public static final double kElevatorL3AlgaePos = 27.8; //Was 26
public static final double kElevatorClimbPos = 0;


// Algae is -0.441 to 3
public static final double kAlgaeOutSpinSpeed = -20;
public static final double kAlgaeInSpinSpeed = 30;
public static final double kAlgaeIdleSpinSpeed = 10; //10
public static final double kAlgaeStopSpinSpeed = 0;

public static final double kAlgaeIntakePos = 4.4;
public static final double kAlgaeClimbPos = 3.336;
public static final double kAlgaeNetPos = 1.807;
public static final double kAlgaeHomePos = 1.3; // potentially like 7
public static final double kAlgaeIntakePositionTolerance = 0.05;
public static final double kAlgaeFloorPickupPos = 5.4;




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

  public static final InvertedValue kAlgaeIntakeRotateDirection = InvertedValue.Clockwise_Positive;



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
  public static final PathConstraints kPathContraints = new PathConstraints(4, 3, 3/2 * Math.PI, 2 * Math.PI);
  public static final Time kAutoAlignAdjustTimeout = Seconds.of(1.0);
  

  //Bunch of preset drive positions Xvalue, Yvalue, rotation
  public static final double[] kBlue19R = {3.517652 + kAprilX2Offset, 5.137091 - kAprilYOffset, 300};       //RIGHT FIXED
  public static final double[] kBlue19L = {3.850719 + kAprilX2Offset, 5.301503 - kAprilYOffset, 300};    //FIXED
  public static final double[] kBlue20R = {5.00 + kAprilX2Offset, 5.40 - kAprilYOffset, 240};     //RIGHT FIXED
  public static final double[] kBlue20L = {5.289 + kAprilX2Offset, 5.244 - kAprilYOffset, 240};    //FIXED
  public static final double[] kBlue21R = {5.97 - kAprilX1Offset, 4.246, 180}; 
  public static final double[] kBlue21L = {5.97 - kAprilX1Offset, 3.96, 180}; //Maybe
  public static final double[] kBlue22R = {5.498 - kAprilX2Offset, 2.8877, 120};                  //RIGHT FIXED
  public static final double[] kBlue22L = {5.178 - kAprilX2Offset, 2.749, 120};                 //FIXED
  public static final double[] kBlue17R = {3.976 - kAprilX2Offset, 2.617 + kAprilYOffset, 60};     //RIGHT FIXED
  public static final double[] kBlue17L = {3.646 - kAprilX2Offset, 2.818 + kAprilYOffset, 60};  //FIXED
  public static final double[] kBlue18R = {3.029257 + kAprilX1Offset, 3.810388, 0}; 
  public static final double[] kBlue18L = {3.091596 + kAprilX1Offset, 4.147526, 0}; //Maybe
  public static final double[] kRed9R = {(12.026), 5.17, 120};                            //RIGHT FIXED
  public static final double[] kRed9L = {(11.539), 5.3603, 120};                             //FIXED
  public static final double[] kRed8R = {(14.1), 5.17, 60};                               //RIGHT FIXED
  public static final double[] kRed8L = {(13.825), 5.3603, 60};                              //FIXED
  public static final double[] kRed7R = {(14.56), 4.362, 0}; 
  public static final double[] kRed7L = {(14.587), 4.04, 0};                                //FIXED
  public static final double[] kRed6R = {(14.1), 2.91, 300};                           //RIGHT FIXED
  public static final double[] kRed6L = {(13.825), 2.7197, 300};                          //THIS IS THE OG FIXED 
  public static final double[] kRed11R = {(12.026), 2.91, 240};                         //RIGHT FIXED
  public static final double[] kRed11L = {(14.587), 2.7197, 240};                          //FIXED
  public static final double[] kRed10R = {(11.539), 3.693, 180};
  public static final double[] kRed10L = {(11.539), 4.04, 180};                               //FIXED
}
/*
 * Math
 * Center of Red Reef:
 * x = 13.063
 * y = 4.04
 * 
 * For scoring on right Branch
 * (14.1-13.063)^2 + (4.04-2.91)^2
 * r = 2.3527
 * Standard X change = 1.037
 * Standard Y change = 1.13
 * 
 * 
 * For scoring on left Branch
 * Standard X change = 0.762
 * Standard Y change = 1.3203
 * (13.825-13.063)^2 + (4.04-2.7197)^2
 * r = 1.5244133593 for all red(?) left scores
 */