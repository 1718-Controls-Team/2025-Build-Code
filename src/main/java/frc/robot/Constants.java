// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * public static final double kShooterIntakeRotateMaxForwardVoltage = 11;
    public static final double kShooterIntakeRotateMaxReverseVoltage = -11;
    public static final double kShooterIntakeRotateSupplyCurrentLimit = 1; //was 20.  
    public static final double kShooterIntakeRotateVoltageClosedLoopRampPeriod = 0.1;
    public static final InvertedValue kShooterIntakeRotateDirection = InvertedValue.CounterClockwise_Positive;
    public static final double kShooterIntakeRotateMotionMagicCruiseVelocity = 0;
    public static final double kShooterIntakeRotateMotionMagicAcceleration = 0;
 */
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


// coralRotate Constants
  public static final double kCoralRotateProportional = 0; 
  public static final double kCoralRotateIntegral = 0;
  public static final double kCoralRotateDerivative = 0;
  public static final double kCoralRotateFeedFoward = 0;
  public static final double kCoralRotateVelocityFeedForward = 0;
  public static final double kCoralRotateGravityFeedForward = 0;
  public static final double kCoralMaxReverseVoltage = -11;
  public static final double kCoralMaxForwardVoltage = 11;
  public static final double kCoralRotateMotionMagicAcceleration = 0;
  public static final double kCoralRotateMotionMagicCruiseVelocity = 0;
  public static final double kCoralRotateVoltageClosedLoopRampPeriod = 0;
  public static final double kCoralRotateSupplyCurrentLimit = 1;
  public static final InvertedValue kCoralRotateDirection = InvertedValue.Clockwise_Positive;
  public static final SensorDirectionValue kCoralRotateCancoderDirection = SensorDirectionValue.Clockwise_Positive;
  public static final double kCoralCancoderAbsoluteSensorDiscontinuityPoint = 1;
  public static final double kCoralRotateRotorToSensorRatio = 0;
// these constants below are not accurate lmao
  public static final double kCoralIntakeDownPos = 0;
  public static final double kCoralIntakeSpeed = 0;
  public static final double kCoralSpinIntakeSpeed = 0;
  public static final double kCoralSpinStopSpeed = 0;

// coralSpin Constants
  public static final double kCoralSpinDerivative = 0;
  public static final double kCoralSpinIntegral = 0;
  public static final double kCoralSpinMaxForwardVoltage = 11;
  public static final double kCoralSpinMaxReverseVoltage = -11;
  public static final double kCoralSpinProportional = 0;
  public static final InvertedValue kCoralSpinDirection = InvertedValue.Clockwise_Positive;
  public static final double kCoralSpinSupplyCurrentLimit = 0;
  public static final double kCoralSpinVelocityFeedFoward = 0;
  public static final double kCoralSpinVoltageClosedLoopRampPeriod = 0;

// Climber Constants
  public static final double kTClimberProportional = 0; 
  public static final double kTClimberIntegral = 0;
  public static final double kTClimberDerivative = 0;
  public static final double kTClimberFeedFoward = 0;
  public static final double kTClimberVelocityFeedForward = 0;
  public static final double kTClimberGravityFeedForward = 0;
  public static final double kTClimberMaxReverseVoltage = -11;
  public static final double kTClimberMaxForwardVoltage = 11;
  public static final double kTClimberMotionMagicAcceleration = 0;
  public static final double kTClimberMotionMagicCruiseVelocity = 0;
  public static final double kTClimberVoltageClosedLoopRampPeriod = 0;
  public static final double kTClimberSupplyCurrentLimit = 1;
  public static final InvertedValue kTClimberDirection = InvertedValue.Clockwise_Positive;
  public static final SensorDirectionValue kTClimberCancoderDirection = SensorDirectionValue.Clockwise_Positive;
  public static final double kTClimberCancoderAbsoluteSensorDiscontinuityPoint = 1;
  public static final double kTClimberRotorToSensorRatio = 0;
  public static final double kTClimberVelocityFeedFoward = 0;
  // not skigma, not alphica aproved constants below
  public static final double kTClimberUpPos = 0;


// Elevator Constants
  public static final double kElevator1Proportional = 0;
  public static final double kElevator1Integral = 0;
  public static final double kElevator1Derivative = 0;
  public static final double kElevator1VelocityFeedFoward = 0;
  public static final InvertedValue kElevator1Direction = InvertedValue.Clockwise_Positive;
  public static final double kElevator1SupplyCurrentLimit = 0;
  public static final double kElevator1VoltageClosedLoopRampPeriod = 0;
  public static final double kElevator1MaxForwardVoltage = 11;
  public static final double kElevator1MaxReverseVoltage = -11;

  public static final double kElevator2Proportional = 0;
  public static final double kElevator2Integral = 0;
  public static final double kElevator2Derivative = 0;
  public static final double kElevator2VelocityFeedFoward = 0;
  public static final InvertedValue kElevator2Direction = InvertedValue.Clockwise_Positive;
  public static final double kElevator2SupplyCurrentLimit = 0;
  public static final double kElevator2VoltageClosedLoopRampPeriod = 0;
  public static final double kElevator2MaxForwardVoltage = 11;
  public static final double kElevator2MaxReverseVoltage = -11;
// These Constants below arent real take yo pills
  public static final double kElevatorIntakePos = 0;
  public static final double kElevatorHomePos = 0;
  public static final double kElevatorAlgaePos = 0;

// Algae Intake Spins Constants
  public static final double kAlgaeIntake1Derivative = 0;
  public static final double kAlgaeIntake1Integral = 0;
  public static final double kAlgaeIntake1MaxForwardVoltage = 11;
  public static final double kAlgaeIntake1MaxReverseVoltage = -11;
  public static final double kAlgaeIntake1Proportional = 0;
  public static final InvertedValue kAlgaeIntake1Direction = InvertedValue.Clockwise_Positive;
  public static final double kAlgaeIntake1SupplyCurrentLimit = 0;
  public static final double kAlgaeIntake1VelocityFeedFoward = 0;
  public static final double kAlgaeIntake1VoltageClosedLoopRampPeriod = 0;

  public static final double kAlgaeIntake2Derivative = 0;
  public static final double kAlgaeIntake2Integral = 0;
  public static final double kAlgaeIntake2MaxForwardVoltage = 11;
  public static final double kAlgaeIntake2MaxReverseVoltage = -11;
  public static final double kAlgaeIntake2Proportional = 0;
  public static final InvertedValue kAlgaeIntake2Direction = InvertedValue.Clockwise_Positive;
  public static final double kAlgaeIntake2SupplyCurrentLimit = 0;
  public static final double kAlgaeIntake2VelocityFeedFoward = 0;
  public static final double kAlgaeIntake2VoltageClosedLoopRampPeriod = 0;

  public static final double kAlgaeIntakeRotateProportional = 0; 
  public static final double kAlgaeIntakeRotateIntegral = 0;
  public static final double kAlgaeIntakeRotateDerivative = 0;
  public static final double kAlgaeIntakeRotateFeedFoward = 0;
  public static final double kAlgaeIntakeRotateVelocityFeedForward = 0;
  public static final double kAlgaeIntakeRotateGravityFeedForward = 0;
  public static final double kAlgaeIntakeRotateMaxReverseVoltage = -11;
  public static final double kAlgaeIntakeRotateMaxForwardVoltage = 11;
  public static final double kAlgaeIntakeRotateMotionMagicAcceleration = 0;
  public static final double kAlgaeIntakeRotateMotionMagicCruiseVelocity = 0;
  public static final double kAlgaeIntakeRotateVoltageClosedLoopRampPeriod = 0;
  public static final double kAlgaeIntakeRotateSupplyCurrentLimit = 1;
  public static final InvertedValue kAlgaeIntakeRotateDirection = InvertedValue.Clockwise_Positive;
  public static final SensorDirectionValue kAlgaeIntakeRotateCancoderDirection = SensorDirectionValue.Clockwise_Positive;
  public static final double kAlgaeIntakeRotateCancoderAbsoluteSensorDiscontinuityPoint = 1;
  public static final double kAlgaeIntakeRotateRotorToSensorRatio = 0;

  // these constants are fake, not sigma :/
  public static final double kAlgaeOutSpinSpeed = 0;
  public static final double kAlgaeInSpinSpeed = 0;
  public static final double kAlgaeStopSpinSpeed = 0;
  public static final double kAlgaeIntakePos = 0;
  public static final double kAlgaeHomePos = 0;
  public static final double kAlgaeStopSpeed = 0;

  // BeamBreak Constants
  public static final double kIntakeBeamBreakCrossover = 0.9;
  public static final double kShooterBeamBreakCrossover = 0.9;
  public static final boolean kPrintSubsystemBeamBreak = false;
  public static final int kBeamBreakZeroAnalog = 0;
  public static final int kBeamBreakOneAnalog = 1;

  //Custom Brownout voltage for the RIO2.
  public static final double kCustomBrownout = 6.0;
}
