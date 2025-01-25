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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class CoralIntake extends SubsystemBase {
  /** Creates a new CoralIntake. */
  TalonFX m_coralSpin = new TalonFX(17);
  TalonFX m_coralRotate = new TalonFX(15);
  CANcoder m_coralRotatCancoder = new CANcoder(16);


  private final VelocityVoltage coralSpinVelocityRequest = new VelocityVoltage(0);
  private final MotionMagicVoltage coralRotationRequest = new MotionMagicVoltage(0);
  private final DutyCycleOut coralRotateVoltageRequest = new DutyCycleOut(0);
 
  public CoralIntake() {
    this.coralIntakeConfiguration(m_coralSpin);
    this.coralRotateConfiguration(m_coralRotate);
    this.configureCoralCancoder(m_coralRotatCancoder);

  }

  public void coralIntakeConfiguration(TalonFX m_coralSpin) {
    TalonFXConfiguration coralSpinConfig = new TalonFXConfiguration();
    Slot0Configs slot0 = coralSpinConfig.Slot0;
      slot0.kP = Constants.kCoralSpinProportional; // An error of 1 rotation per second results in 2V output
      slot0.kI = Constants.kCoralSpinIntegral; // An error of 1 rotation per second increases output by 0.5V every second
      slot0.kD = Constants.kCoralSpinDerivative; // A change of 1 rotation per second squared results in 0.01 volts output
      slot0.kV = Constants.kCoralSpinVelocityFeedFoward; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second Peak output of 8 volts
    coralSpinConfig.Voltage.PeakForwardVoltage = Constants.kCoralSpinMaxForwardVoltage;
    coralSpinConfig.Voltage.PeakReverseVoltage = Constants.kCoralSpinMaxReverseVoltage;
    coralSpinConfig.CurrentLimits.SupplyCurrentLimit = Constants.kCoralSpinSupplyCurrentLimit;
    coralSpinConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kCoralSpinVoltageClosedLoopRampPeriod;
    coralSpinConfig.MotorOutput.Inverted = Constants.kCoralSpinDirection;
    coralSpinConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  }
  public void configureCoralCancoder(CANcoder coralRotateCancoder){  
    CANcoderConfiguration coralRotateCANcoderConfig = new CANcoderConfiguration();
    coralRotateCANcoderConfig.MagnetSensor.MagnetOffset = -0.398926;
    coralRotateCANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = Constants.kCoralCancoderAbsoluteSensorDiscontinuityPoint;
    coralRotateCANcoderConfig.MagnetSensor.SensorDirection = Constants.kCoralRotateCancoderDirection;
  }
  public void coralRotateConfiguration(TalonFX m_coralRotate) {
    TalonFXConfiguration coralRotateConfig = new TalonFXConfiguration();

  coralRotateConfig.MotionMagic.MotionMagicAcceleration = Constants.kCoralRotateMotionMagicAcceleration;
  coralRotateConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.kCoralRotateMotionMagicCruiseVelocity;
  coralRotateConfig.MotorOutput.Inverted = Constants.kCoralRotateDirection;
  coralRotateConfig.CurrentLimits.SupplyCurrentLimit = Constants.kCoralRotateSupplyCurrentLimit;
  coralRotateConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kCoralRotateVoltageClosedLoopRampPeriod;
  coralRotateConfig.Voltage.PeakForwardVoltage = Constants.kCoralMaxReverseVoltage;
  coralRotateConfig.Voltage.PeakForwardVoltage = Constants.kCoralMaxForwardVoltage;

  coralRotateConfig.Feedback.FeedbackRemoteSensorID = Constants.kCoralRotateCancoderCanID;
  coralRotateConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
  coralRotateConfig.Feedback.RotorToSensorRatio = Constants.kCoralRotateRotorToSensorRatio;

  Slot0Configs slot0 = coralRotateConfig.Slot0;
    slot0.kP = Constants.kCoralRotateProportional;
    slot0.kI = Constants.kCoralRotateIntegral;
    slot0.kD = Constants.kCoralRotateDerivative;
    slot0.kG = Constants.kCoralRotateGravityFeedForward;
    slot0.kV = Constants.kCoralRotateVelocityFeedForward;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;

StatusCode coralRotStatus = StatusCode.StatusCodeNotInitialized;
for(int i = 0; i < 5; ++i) {
  coralRotStatus = m_coralRotate.getConfigurator().apply(coralRotateConfig);
  if (coralRotStatus.isOK()) break;
}
if (!coralRotStatus.isOK()) {
  System.out.println("Could not configure device. Error: " + coralRotStatus.toString());
}  

  }

  /**
   * Example command factory method.
   *
   * @return a command 
   */
 
  public void setcoralSpinPower(double speed) {
    m_coralSpin.setControl(coralSpinVelocityRequest.withVelocity(speed));
  
  }
  public void setcoralRotate(double position) {
    m_coralRotate.setControl(coralRotationRequest.withPosition(position));

  }

  public double getCoralPosition() {
    return m_coralSpin.getPosition().getValueAsDouble();
  }
  public void ZeroCoralRotate() {
    m_coralRotate.setControl(coralRotateVoltageRequest);
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
