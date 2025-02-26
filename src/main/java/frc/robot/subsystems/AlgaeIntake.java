// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class AlgaeIntake extends SubsystemBase {

  TalonFX AlgaeRotate = new TalonFX(18, "rio");
  TalonFX AlgaeIntake1 = new TalonFX(20, "rio");
  TalonFX AlgaeIntake2 = new TalonFX(21, "rio");

  
  PositionVoltage AlgaeIntakePosition = new PositionVoltage(0);
  DutyCycleOut AlgaeIntakeVoltage = new DutyCycleOut(0);

  private VelocityVoltage AlgaeVelocityRequest = new VelocityVoltage(0);

  private double AlgaeIntakeRotateDesiredPos = 0;

  public AlgaeIntake() {
    this.configureAlgaeIntakeRotate(AlgaeRotate);
    this.configureAlgaeIntake1(AlgaeIntake1);
    this.configureAlgaeIntake2(AlgaeIntake2);
  }
//############################################## BEGIN WRITING CLASS FUNCTIONS ######################################################


  public void setAlgaeSpinPower(double AlgaeIntakePower) {
    AlgaeIntake1.setControl(AlgaeVelocityRequest.withVelocity(AlgaeIntakePower));
    AlgaeIntake2.setControl(AlgaeVelocityRequest.withVelocity(AlgaeIntakePower));
  }
  

  public void setAlgaeRotatePos(double AlgaeIntakeDesiredPosition) {
    AlgaeRotate.setControl(AlgaeIntakePosition.withPosition(AlgaeIntakeDesiredPosition));
    AlgaeIntakeRotateDesiredPos = AlgaeIntakeDesiredPosition;
  }


  public void ZeroAlgaeRotateOutput() {
    AlgaeRotate.setControl(AlgaeIntakeVoltage);
  }


  public boolean getAlgaeRotateInPosition(){
    if ((Math.abs(AlgaeRotate.getPosition().getValueAsDouble() - AlgaeIntakeRotateDesiredPos) < Constants.kAlgaeIntakePositionTolerance)){
      return true;
    } else {
      return false;
    }
  }

//######################################### Start OF ALGAE CONFIGURATION ######################################################
//######################################### Start OF ALGAE CONFIGURATION ######################################################
//######################################### Start OF ALGAE CONFIGURATION ###################################################### 

//######################################### ALGAE ROTATE CONFIGURATION ###################################################### 


  public void configureAlgaeIntakeRotate(TalonFX algaeIntakeSpin){
    TalonFXConfiguration algaeIntakeRotateConfig = new TalonFXConfiguration();

    algaeIntakeRotateConfig.CurrentLimits.SupplyCurrentLimit = Constants.kAlgaeIntakeRotateSupplyCurrentLimit;
    algaeIntakeRotateConfig.CurrentLimits.SupplyCurrentLimitEnable = true;    

    algaeIntakeRotateConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kAlgaeIntakeRotateVoltageClosedLoopRampPeriod;
    algaeIntakeRotateConfig.Voltage.PeakForwardVoltage = Constants.kAlgaeIntakeRotateMaxForwardVoltage;
    algaeIntakeRotateConfig.Voltage.PeakReverseVoltage = Constants.kAlgaeIntakeRotateMaxReverseVoltage;

    algaeIntakeRotateConfig.MotorOutput.Inverted = Constants.kAlgaeIntakeRotateDirection;
    algaeIntakeRotateConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  

    Slot0Configs slot0 = algaeIntakeRotateConfig.Slot0;
    slot0.kP = Constants.kAlgaeIntakeRotateProportional;
    slot0.kI = Constants.kAlgaeIntakeRotateIntegral;
    slot0.kD = Constants.kAlgaeIntakeRotateDerivative;
    
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kG = Constants.kAlgaeIntakeRotateGravityFeedForward;
    slot0.kV = Constants.kAlgaeIntakeRotateVelocityFeedForward;
    slot0.kS = Constants.kAlgaeIntakeRotateStaticFeedForward;
    

    
    StatusCode algaeIntakeRotateStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      algaeIntakeRotateStatus = AlgaeRotate.getConfigurator().apply(algaeIntakeRotateConfig);
      if (algaeIntakeRotateStatus.isOK()) break;
    }
    if (!algaeIntakeRotateStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + algaeIntakeRotateStatus.toString());
    }
  }

//######################################### ALGAE SPIN 1 CONFIGURATION ###################################################### 


  public void configureAlgaeIntake1(TalonFX algaeIntake1){
    TalonFXConfiguration algaeIntake1VelocityConfig = new TalonFXConfiguration();
    algaeIntake1VelocityConfig.Slot0.kP = Constants.kAlgaeIntake1Proportional; // An error of 1 rotation per second results in 2V output
    algaeIntake1VelocityConfig.Slot0.kI = Constants.kAlgaeIntake1Integral; // An error of 1 rotation per second increases output by 0.5V every second
    algaeIntake1VelocityConfig.Slot0.kD = Constants.kAlgaeIntake1Derivative; // A change of 1 rotation per second squared results in 0.01 volts output
    
    algaeIntake1VelocityConfig.Slot0.kV = Constants.kAlgaeIntake1VelocityFeedForward; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second Peak output of 8 volts
    algaeIntake1VelocityConfig.Slot0.kG = Constants.kAlgaeIntake1GravityFeedForward;
    algaeIntake1VelocityConfig.Slot0.kS = Constants.kAlgaeIntake1StaticFeedForward;

    algaeIntake1VelocityConfig.Voltage.PeakForwardVoltage = Constants.kAlgaeIntake1MaxForwardVoltage;
    algaeIntake1VelocityConfig.Voltage.PeakReverseVoltage = Constants.kAlgaeIntake1MaxReverseVoltage;
    algaeIntake1VelocityConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kAlgaeIntake1VoltageClosedLoopRampPeriod;
    algaeIntake1VelocityConfig.CurrentLimits.SupplyCurrentLimit = Constants.kAlgaeIntake1SupplyCurrentLimit;
    algaeIntake1VelocityConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    algaeIntake1VelocityConfig.MotorOutput.Inverted = Constants.kAlgaeIntake1Direction;
    

    StatusCode algaeIntake1Status = StatusCode.StatusCodeNotInitialized;

    for(int i = 0; i < 5; ++i) {
      algaeIntake1Status = algaeIntake1.getConfigurator().apply(algaeIntake1VelocityConfig);
      if (algaeIntake1Status.isOK()) break;
    }
    if (!algaeIntake1Status.isOK()) {
      System.out.println("Could not configure device. Error: " + algaeIntake1Status.toString());
    }
  }

//######################################### ALGAE SPIN 2 CONFIGURATION ###################################################### 


public void configureAlgaeIntake2(TalonFX algaeIntake2){
    TalonFXConfiguration algaeIntake2VelocityConfig = new TalonFXConfiguration();
    algaeIntake2VelocityConfig.Slot0.kP = Constants.kAlgaeIntake2Proportional; // An error of 1 rotation per second results in 2V output
    algaeIntake2VelocityConfig.Slot0.kI = Constants.kAlgaeIntake2Integral; // An error of 1 rotation per second increases output by 0.5V every second
    algaeIntake2VelocityConfig.Slot0.kD = Constants.kAlgaeIntake2Derivative; // A change of 1 rotation per second squared results in 0.01 volts output
    
    algaeIntake2VelocityConfig.Slot0.kV = Constants.kAlgaeIntake2VelocityFeedForward; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second Peak output of 8 volts
    algaeIntake2VelocityConfig.Slot0.kG = Constants.kAlgaeIntake2GravityFeedForward;
    algaeIntake2VelocityConfig.Slot0.kS = Constants.kAlgaeIntake2StaticFeedForward;

    algaeIntake2VelocityConfig.Voltage.PeakForwardVoltage = Constants.kAlgaeIntake2MaxForwardVoltage;
    algaeIntake2VelocityConfig.Voltage.PeakReverseVoltage = Constants.kAlgaeIntake2MaxReverseVoltage;
    algaeIntake2VelocityConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kAlgaeIntake2VoltageClosedLoopRampPeriod;

    algaeIntake2VelocityConfig.CurrentLimits.SupplyCurrentLimit = Constants.kAlgaeIntake2SupplyCurrentLimit;
    algaeIntake2VelocityConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    algaeIntake2VelocityConfig.MotorOutput.Inverted = Constants.kAlgaeIntake2Direction;
    

    StatusCode algaeIntake2Status = StatusCode.StatusCodeNotInitialized;

    for(int i = 0; i < 5; ++i) {
      algaeIntake2Status = algaeIntake2.getConfigurator().apply(algaeIntake2VelocityConfig);
      if (algaeIntake2Status.isOK()) break;
    }
    if (!algaeIntake2Status.isOK()) {
      System.out.println("Could not configure device. Error: " + algaeIntake2Status.toString());
    }
  }
//################################################### END OF ALGAE CONFIGURATION #######################################################
//################################################### END OF ALGAE CONFIGURATION #######################################################
//################################################### END OF ALGAE CONFIGURATION ####################################################### 



  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
