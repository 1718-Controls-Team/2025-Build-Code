// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class AlgaeIntake extends SubsystemBase {
  /** Creates a new AlgaeIntake. */
  TalonFX AlgaeRotate = new TalonFX(18);
  CANcoder AlgaeRotateCancoder = new CANcoder(19);
  TalonFX AlgaeIntake1 = new TalonFX(20);
  TalonFX AlgaeIntake2 = new TalonFX(21);
  
  MotionMagicVoltage AlgaeIntakePosition = new MotionMagicVoltage(0);
  VelocityVoltage AlgaeIntake1Power = new VelocityVoltage(0);
  VelocityVoltage AlgaeIntake2Power = new VelocityVoltage(0);
  DutyCycleOut AlgaeIntakeVoltage = new DutyCycleOut(0);

  private final VelocityVoltage AlgaeVelocityRequest = new VelocityVoltage(0);

  public AlgaeIntake() {
    this.configureAlgaeIntakeRotate(AlgaeRotate);
    this.configureAlgaeIntakeCancoder(AlgaeRotateCancoder);
    this.configureAlgaeIntake1(AlgaeIntake1);
    this.configureAlgaeIntake2(AlgaeIntake2);
  }

  public void configureAlgaeIntakeRotate(TalonFX algaeIntakeSpin){
    TalonFXConfiguration algaeIntakeRotateConfig = new TalonFXConfiguration();

    algaeIntakeRotateConfig.CurrentLimits.SupplyCurrentLimit = Constants.kAlgaeIntakeRotateSupplyCurrentLimit;
    algaeIntakeRotateConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kAlgaeIntakeRotateVoltageClosedLoopRampPeriod;
    algaeIntakeRotateConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    algaeIntakeRotateConfig.MotorOutput.Inverted = Constants.kAlgaeIntakeRotateDirection;
    algaeIntakeRotateConfig.Voltage.PeakForwardVoltage = Constants.kAlgaeIntakeRotateMaxForwardVoltage;
    algaeIntakeRotateConfig.Voltage.PeakReverseVoltage = Constants.kAlgaeIntakeRotateMaxReverseVoltage;
    algaeIntakeRotateConfig.MotionMagic.MotionMagicAcceleration = Constants.kAlgaeIntakeRotateMotionMagicAcceleration;
    algaeIntakeRotateConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.kAlgaeIntakeRotateMotionMagicCruiseVelocity;
    //algaeIntakeRotateConfig.MotionMagic.MotionMagicJerk = Constants.kAlgaeIntakeRotateMotionMagicJerk; // Idk how needed Jerk is
    algaeIntakeRotateConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    Slot0Configs slot0 = algaeIntakeRotateConfig.Slot0;
    slot0.kP = Constants.kAlgaeIntakeRotateProportional;
    slot0.kI = Constants.kAlgaeIntakeRotateIntegral;
    slot0.kD = Constants.kAlgaeIntakeRotateDerivative;
    slot0.kG = Constants.kAlgaeIntakeRotateGravity;
    slot0.kV = Constants.kAlgaeIntakeRotateVelocityFeedFoward;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;

    //slot0.kV = Constants.kAlgaeIntakeRotateVelocityFeedFoward;
    //slot0.kS = Constants.kAlgaeIntakeRotateStaticFeedFoward; // The value of s is approximately the number of volts needed to get the mechanism moving
    algaeIntakeRotateConfig.Feedback.FeedbackRemoteSensorID = Constants.kAlgaeIntakeRotateCancoderCanID;
    algaeIntakeRotateConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    algaeIntakeRotateConfig.Feedback.RotorToSensorRatio = Constants.kAlgaeIntakeRotateRotorToSensorRatio;

    //Setting the config option that allows playing music on the motor during disabled.
    algaeIntakeRotateConfig.Audio.AllowMusicDurDisable = true;

    StatusCode algaeIntakeRotateStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      algaeIntakeRotateStatus = m_algaeIntakeRotate.getConfigurator().apply(algaeIntakeRotateConfig);
      if (algaeIntakeRotateStatus.isOK()) break;
    }
    if (!algaeIntakeRotateStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + algaeIntakeRotateStatus.toString());
    }
  }

  public void configureAlgaeIntakeCancoder(CANcoder algaeIntakeCancoder){  
    CANcoderConfiguration algaeIntakeRotateCANcoderConfig = new CANcoderConfiguration();
    algaeIntakeRotateCANcoderConfig.MagnetSensor.MagnetOffset = -0.398926;
    algaeIntakeRotateCANcoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    algaeIntakeRotateCANcoderConfig.MagnetSensor.SensorDirection = Constants.kAlgaeIntakeRotateCancoderDirection;
  }

  public void configureAlgaeIntake1(TalonFX algaeIntake1){
    TalonFXConfiguration algaeIntake1VelocityConfig = new TalonFXConfiguration();
    algaeIntake1VelocityConfig.Slot0.kP = Constants.kAlgaeIntake1Proportional; // An error of 1 rotation per second results in 2V output
    algaeIntake1VelocityConfig.Slot0.kI = Constants.kAlgaeIntake1Integral; // An error of 1 rotation per second increases output by 0.5V every second
    algaeIntake1VelocityConfig.Slot0.kD = Constants.kAlgaeIntake1Derivative; // A change of 1 rotation per second squared results in 0.01 volts output
    algaeIntake1VelocityConfig.Slot0.kV = Constants.kAlgaeIntake1VelocityFeedFoward; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second Peak output of 8 volts
    algaeIntake1VelocityConfig.Voltage.PeakForwardVoltage = Constants.kAlgaeIntake1MaxForwardVoltage;
    algaeIntake1VelocityConfig.Voltage.PeakReverseVoltage = Constants.kAlgaeIntake1MaxReverseVoltage;
    algaeIntake1VelocityConfig.CurrentLimits.SupplyCurrentLimit = Constants.kAlgaeIntake1SupplyCurrentLimit;
    algaeIntake1VelocityConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kAlgaeIntake1VoltageClosedLoopRampPeriod;
    algaeIntake1VelocityConfig.MotorOutput.Inverted = Constants.kAlgaeIntake1Direction;
    algaeIntake1VelocityConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    //Setting the config option that allows playing music on the motor during disabled.
    algaeIntake1VelocityConfig.Audio.AllowMusicDurDisable = true;

    StatusCode algaeIntake1Status = StatusCode.StatusCodeNotInitialized;

    for(int i = 0; i < 5; ++i) {
      algaeIntake1Status = algaeIntake1.getConfigurator().apply(algaeIntake1VelocityConfig);
      if (algaeIntake1Status.isOK()) break;
    }
    if (!algaeIntake1Status.isOK()) {
      System.out.println("Could not configure device. Error: " + algaeIntake1Status.toString());
    }
  }

public void configureAlgaeIntake2(TalonFX algaeIntake2){
    TalonFXConfiguration algaeIntake2VelocityConfig = new TalonFXConfiguration();
    algaeIntake2VelocityConfig.Slot0.kP = Constants.kAlgaeIntake2Proportional; // An error of 1 rotation per second results in 2V output
    algaeIntake2VelocityConfig.Slot0.kI = Constants.kAlgaeIntake2Integral; // An error of 1 rotation per second increases output by 0.5V every second
    algaeIntake2VelocityConfig.Slot0.kD = Constants.kAlgaeIntake2Derivative; // A change of 1 rotation per second squared results in 0.01 volts output
    algaeIntake2VelocityConfig.Slot0.kV = Constants.kAlgaeIntake2VelocityFeedFoward; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second Peak output of 8 volts
    algaeIntake2VelocityConfig.Voltage.PeakForwardVoltage = Constants.kAlgaeIntake2MaxForwardVoltage;
    algaeIntake2VelocityConfig.Voltage.PeakReverseVoltage = Constants.kAlgaeIntake2MaxReverseVoltage;
    algaeIntake2VelocityConfig.CurrentLimits.SupplyCurrentLimit = Constants.kAlgaeIntake2SupplyCurrentLimit;
    algaeIntake2VelocityConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kAlgaeIntake2VoltageClosedLoopRampPeriod;
    algaeIntake2VelocityConfig.MotorOutput.Inverted = Constants.kAlgaeIntake2Direction;
    algaeIntake2VelocityConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    //Setting the config option that allows playing music on the motor during disabled.
    algaeIntake2VelocityConfig.Audio.AllowMusicDurDisable = true;

    StatusCode algaeIntake2Status = StatusCode.StatusCodeNotInitialized;

    for(int i = 0; i < 5; ++i) {
      algaeIntake2Status = algaeIntake2.getConfigurator().apply(algaeIntake2VelocityConfig);
      if (algaeIntake2Status.isOK()) break;
    }
    if (!algaeIntake2Status.isOK()) {
      System.out.println("Could not configure device. Error: " + algaeIntake2Status.toString());
    }
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
 
  public void setAlgaeSpinPower(double AlgaeIntakePower) {
    AlgaeIntake1.setControl(AlgaeVelocityRequest.withVelocity(AlgaeIntakePower));
    AlgaeIntake2.setControl(AlgaeVelocityRequest.withVelocity(AlgaeIntakePower));
    //System.out.println(AlgaeIntakePower);
  }
  
  public void setAlgaeRotatePosition(double AlgaeIntakeDesiredPosition) {
    AlgaeRotate.setControl(AlgaeIntakePosition.withPosition(AlgaeIntakeDesiredPosition));
  }

  public void ZeroAlgaeRotateOutput() {
    AlgaeRotate.setControl(AlgaeIntakeVoltage);
  }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
