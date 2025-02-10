// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class TClimber extends SubsystemBase {
  /** Creates a new TClimber. */
  TalonFX m_TClimber = new TalonFX(22);

  
  private final MotionMagicVoltage TClimberRotationRequest = new MotionMagicVoltage(0);
  private final DutyCycleOut TClimberVoltageRequest = new DutyCycleOut(0);

  public TClimber() {
    this.configureTClimber(m_TClimber);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
 
  public void setTClimberPosition(double DesiredPosition) {
    m_TClimber.setControl(TClimberRotationRequest.withPosition(DesiredPosition));
    //System.out.println(DesiredPosition);
    
  }
  
  public void configureTClimber(TalonFX frontIntakeSpin){
    TalonFXConfiguration TClimberConfig = new TalonFXConfiguration();

    TClimberConfig.CurrentLimits.SupplyCurrentLimit = Constants.kTClimberSupplyCurrentLimit;
    TClimberConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kTClimberVoltageClosedLoopRampPeriod;
    TClimberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    TClimberConfig.MotorOutput.Inverted = Constants.kTClimberDirection;
    TClimberConfig.Voltage.PeakForwardVoltage = Constants.kTClimberMaxForwardVoltage;
    TClimberConfig.Voltage.PeakReverseVoltage = Constants.kTClimberMaxReverseVoltage;
    TClimberConfig.MotionMagic.MotionMagicAcceleration = Constants.kTClimberMotionMagicAcceleration;
    TClimberConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.kTClimberMotionMagicCruiseVelocity;
    //TClimberConfig.MotionMagic.MotionMagicJerk = Constants.kTClimberMotionMagicJerk; // Idk how needed Jerk is
    TClimberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    Slot0Configs slot0 = TClimberConfig.Slot0;
    slot0.kP = Constants.kTClimberProportional;
    slot0.kI = Constants.kTClimberIntegral;
    slot0.kD = Constants.kTClimberDerivative;
    slot0.kG = Constants.kTClimberGravityFeedForward;
    slot0.kV = Constants.kTClimberVelocityFeedFoward;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;

    //slot0.kV = Constants.kTClimberVelocityFeedFoward;
    //slot0.kS = Constants.kTClimberStaticFeedFoward; // The value of s is approximately the number of volts needed to get the mechanism moving
    //Setting the config option that allows playing music on the motor during disabled.
    TClimberConfig.Audio.AllowMusicDurDisable = true;

    StatusCode TClimberStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      TClimberStatus = m_TClimber.getConfigurator().apply(TClimberConfig);
      if (TClimberStatus.isOK()) break;
    }
    if (!TClimberStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + TClimberStatus.toString());
    }
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public void setClimberRotate(double position) {
    m_TClimber.setControl(TClimberRotationRequest.withPosition(position));

  }
  public void ZeroClimberRotate() {
    m_TClimber.setControl(TClimberVoltageRequest);
  }



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
