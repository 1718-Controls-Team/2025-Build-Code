// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

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
  public static final int kCoralRotateCancoderCanID = 16;
  public static final double kCoralRotateRotorToSensorRatio = 0;

// coralSpiin Constants
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
public static final int kTClimberCancoderCanID = 23;

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

  
}
