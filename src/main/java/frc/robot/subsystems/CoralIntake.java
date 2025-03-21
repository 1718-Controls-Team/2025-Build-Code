// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class CoralIntake extends SubsystemBase {

  TalonFX m_coralSpin = new TalonFX(17, "rio");
  TalonFX m_coralRotate = new TalonFX(15, "rio");


  private final VelocityVoltage coralSpinVelocityRequest = new VelocityVoltage(0);
  private final MotionMagicVoltage coralRotationRequest = new MotionMagicVoltage(0);
  private final DutyCycleOut coralRotateVoltageRequest = new DutyCycleOut(0);

  private double CoralIntakeRotateDesiredPos = 0;
  public boolean ScoringL4 = false;
  public boolean Spitting = false;

  public CoralIntake() {
    this.coralIntakeConfiguration(m_coralSpin);
    this.coralRotateConfiguration(m_coralRotate);
  }
  //############################################## BEGIN WRITING CLASS FUNCTIONS ######################################################

  public boolean getSpitting() {
    return Spitting;
  }

  public void setSpitting(boolean spitting) {
    Spitting = spitting;
  }

  public double getCoralRotatePosition() {
    return m_coralRotate.getPosition().getValueAsDouble();
  }

  public void setL4CoralSpitMode(boolean scoringL4) {
    ScoringL4 = scoringL4;
  }

  public boolean getL4CoralSpitMode() {
    return ScoringL4;
  }


  public void setcoralSpinPower(double speed) {
    m_coralSpin.setControl(coralSpinVelocityRequest.withVelocity(speed));
  }


  public void setcoralRotate(double position) {
    m_coralRotate.setControl(coralRotationRequest.withPosition(position));
    CoralIntakeRotateDesiredPos = position;
  }


  public boolean getCoralRotateInPosition() {
    if ((Math.abs(m_coralRotate.getPosition().getValueAsDouble() - CoralIntakeRotateDesiredPos) < Constants.kCoralRotatePositionTolerance)){
      return true;
    } else {
      return false;
    }
  }


  public double getCoralPosition() {
    return m_coralRotate.getPosition().getValueAsDouble();
  }


  public void ZeroCoralRotate() {
    m_coralRotate.setControl(coralRotateVoltageRequest);
  }

  public Command setCoralSpinCommand(double speed){
    return this.runOnce(() -> setcoralSpinPower(speed));
  }


//############################################# Start OF CORAL INTAKE CONFIGURATION ####################################################
//############################################# Start OF CORAL INTAKE CONFIGURATION ####################################################
//############################################# Start OF CORAL INTAKE CONFIGURATION #################################################### 

//############################################# CORAL SPIN CONFIGURATION #################################################### 

public void coralIntakeConfiguration(TalonFX m_coralSpin) {
    TalonFXConfiguration coralSpinConfig = new TalonFXConfiguration();
    
    coralSpinConfig.Voltage.PeakForwardVoltage = Constants.kCoralSpinMaxForwardVoltage;
    coralSpinConfig.Voltage.PeakReverseVoltage = Constants.kCoralSpinMaxReverseVoltage;
    coralSpinConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kCoralSpinVoltageClosedLoopRampPeriod;

    coralSpinConfig.CurrentLimits.SupplyCurrentLimit = Constants.kCoralSpinSupplyCurrentLimit;
    coralSpinConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    coralSpinConfig.MotorOutput.Inverted = Constants.kCoralSpinDirection;

    
    Slot0Configs slot0 = coralSpinConfig.Slot0;
      slot0.kP = Constants.kCoralSpinProportional; // An error of 1 rotation per second results in 2V output
      slot0.kI = Constants.kCoralSpinIntegral; // An error of 1 rotation per second increases output by 0.5V every second
      slot0.kD = Constants.kCoralSpinDerivative; // A change of 1 rotation per second squared results in 0.01 volts output
      slot0.kV = Constants.kCoralSpinVelocityFeedForward; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second Peak output of 8 volts
      slot0.kG = Constants.kCoralSpinGravityFeedForward;
      slot0.kS = Constants.kCoralSpinStaticFeedForward;
  }
  
  //############################################# CORAL ROTATE CONFIGURATION #################################################### 


  public void coralRotateConfiguration(TalonFX m_coralRotate) {
    TalonFXConfiguration coralRotateConfig = new TalonFXConfiguration();

  coralRotateConfig.MotionMagic.MotionMagicAcceleration = Constants.kCoralRotateMotionMagicAcceleration;
  coralRotateConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.kCoralRotateMotionMagicCruiseVelocity;
  coralRotateConfig.MotorOutput.Inverted = Constants.kCoralRotateDirection;
  coralRotateConfig.CurrentLimits.SupplyCurrentLimit = Constants.kCoralRotateSupplyCurrentLimit;
  coralRotateConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kCoralRotateVoltageClosedLoopRampPeriod;
  coralRotateConfig.Voltage.PeakForwardVoltage = Constants.kCoralMaxReverseVoltage;
  coralRotateConfig.Voltage.PeakForwardVoltage = Constants.kCoralMaxForwardVoltage;

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
//############################################### END OF CORAL INTAKE CONFIGURATION ####################################################
//############################################### END OF CORAL INTAKE CONFIGURATION ####################################################
//############################################### END OF CORAL INTAKE CONFIGURATION #################################################### 

  
  public boolean exampleCondition() {
    return false;
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
