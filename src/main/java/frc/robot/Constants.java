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

public static final String kLimelightName = "limelight";


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
  public static final double kCoralRotateDeliveryPos = 0;
  public static final double kCoralOutSpinSpeed = 0;
  public static final double kCoralInSpinSpeed = 0;
  public static final double kCoralStopSpinSpeed = 0;
  public static final double kCoralRotatePositionTolerance = 20;
  public static final double kCoralRotateHomePos = 0;

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
  public static final double kElevatorCoralIntakePos = 0;
  public static final double kElevatorHomePos = 0;
  public static final double kElevatorL2ScoringPos = 0;
  public static final double kElevatorL3ScoringPos = 0;
  public static final double kElevatorL4ScoringPos = 0;

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
  public static final double kAlgaeIntakePositionTolerance = 10;

  // BeamBreak Constants
  public static final double kIntakeBeamBreakCrossover = 0.9;
  public static final double kShooterBeamBreakCrossover = 0.9;
  public static final boolean kPrintSubsystemBeamBreak = false;
  public static final int kBeamBreakZeroAnalog = 0;
  public static final int kBeamBreakOneAnalog = 1;

  //Custom Brownout voltage for the RIO2.
  public static final double kCustomBrownout = 6.0;

<<<<<<< HEAD
  //Bunch of preset drive positions
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
  public static final double[] kRedTopLR = {0, 0, 0};
  public static final double[] kRedTopLL = {0, 0, 0};
  public static final double[] kRedTopRR = {0, 0, 0};
  public static final double[] kRedTopRL = {0, 0, 0};
  public static final double[] kRedRightR = {0, 0, 0};
  public static final double[] kRedRightL = {0, 0, 0};
  public static final double[] kRedBottomRR = {0, 0, 0};
  public static final double[] kRedBottomRL = {0, 0, 0};
  public static final double[] kRedBottomLR = {0, 0, 0};
  public static final double[] kRedBottomLL = {0, 0, 0};
  public static final double[] kRedLeftR = {0, 0, 0};
  public static final double[] kRedLeftL = {0, 0, 0};
=======
  //Bunch of preset drive positions Xvalue, Yvalue, rotation
  public static final double[] kRedTopLR = {(17.55 - kBlueTopRL[1]), kBlueTopRL[2], -60};
  public static final double[] kRedTopLL = {(17.55 - kBlueTopRR[1]), kBlueTopRR[2], -60};
  public static final double[] kRedTopRR = {(17.55 - kBlueTopLL[1]), kBlueTopLL[2], -120};
  public static final double[] kRedTopRL = {(17.55 - kBlueTopLR[1]), kBlueTopLR[2], -120};
  public static final double[] kRedRightR = {(17.55 - kBlueLeftL[1]), kBlueLeftL[2], -180};
  public static final double[] kRedRightL = {(17.55 - kBlueLeftR[1]), kBlueLeftR[2], -180};
  public static final double[] kRedBottomRR = {(17.55 - kBlueBottomLL[1]), kBlueBottomLL[2], 120};
  public static final double[] kRedBottomRL = {(17.55 - kBlueBottomLR[1]), kBlueBottomLR[2], 120};
  public static final double[] kRedBottomLR = {(17.55 - kBlueBottomRL[1]), kBlueBottomRL[2], 60};
  public static final double[] kRedBottomLL = {(17.55 - kBlueBottomRR[1]), kBlueBottomRR[2], 60};
  public static final double[] kRedLeftR = {(17.55 - kBlueRightL[1]), kBlueRightL[2], 0};
  public static final double[] kRedLeftL = {(17.55 - kBlueRightR[1]), kBlueRightR[2], 0};
>>>>>>> 6140d17b0c579d287b394a6caeb4697ed219f76b
}
