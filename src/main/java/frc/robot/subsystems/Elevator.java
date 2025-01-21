// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  TalonFX m_Elevator1 = new TalonFX(13);
  TalonFX m_Elevator2 = new TalonFX(14);

  private final PositionVoltage ElevatorPositionRequest = new PositionVoltage(0).withSlot(0);
  private final DutyCycleOut ElevatorVoltageRequest = new DutyCycleOut(0);

  public Elevator() {
    this.configureElevator1(m_Elevator1);
    this.configureElevator2(m_Elevator2);
  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
 
  public void setElevatorPosition(double DesiredPosition) {
    m_Elevator1.setControl(ElevatorPositionRequest.withPosition(DesiredPosition));
    m_Elevator2.setControl(ElevatorPositionRequest.withPosition(DesiredPosition));
    //System.out.println(Erika);
    
  }

  public void setClimberZeroOutput() {
    m_Elevator1.setControl(ElevatorVoltageRequest);
    m_Elevator2.setControl(ElevatorVoltageRequest);
  }

  public void configureElevator1(TalonFX elevator1){
    //Start Configuring Elevator
    TalonFXConfiguration elevator1Config = new TalonFXConfiguration();

    elevator1Config.MotorOutput.Inverted = Constants.kElevator1Direction;
    elevator1Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevator1Config.CurrentLimits.SupplyCurrentLimit = Constants.kElevator1SupplyCurrentLimit;
    elevator1Config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kElevator1VoltageClosedLoopRampPeriod;
    elevator1Config.Voltage.PeakForwardVoltage = Constants.kElevator1MaxForwardVoltage;
    elevator1Config.Voltage.PeakReverseVoltage = Constants.kElevator1MaxReverseVoltage;
    elevator1Config.CurrentLimits.SupplyCurrentLimitEnable = true;

    Slot0Configs slot0 = elevator1Config.Slot0;
    slot0.kP = Constants.kElevator1Proportional;
    slot0.kI = Constants.kElevator1Integral;
    slot0.kD = Constants.kElevator1Derivative;
    slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kV = Constants.kElevator1VelocityFeedFoward;
    //slot0.kS = Constants.kElevator1StaticFeedFoward; // The value of s is approximately the number of volts needed to get the mechanism moving

    //Setting the config option that allows playing music on the motor during disabled.
    //elevator1Config.Audio.AllowMusicDurDisable = true;
 
    StatusCode climberStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      climberStatus = elevator1.getConfigurator().apply(elevator1Config);
      if (climberStatus.isOK()) break;
    }
    if (!climberStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + climberStatus.toString());
    }
    m_Elevator1.setPosition(0);
  }

  public void configureElevator2(TalonFX elevator2){
    //Start Configuring Elevator
    TalonFXConfiguration elevator2Config = new TalonFXConfiguration();

    elevator2Config.MotorOutput.Inverted = Constants.kElevator1Direction;
    elevator2Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevator2Config.CurrentLimits.SupplyCurrentLimit = Constants.kElevator1SupplyCurrentLimit;
    elevator2Config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kElevator1VoltageClosedLoopRampPeriod;
    elevator2Config.Voltage.PeakForwardVoltage = Constants.kElevator1MaxForwardVoltage;
    elevator2Config.Voltage.PeakReverseVoltage = Constants.kElevator1MaxReverseVoltage;
    elevator2Config.CurrentLimits.SupplyCurrentLimitEnable = true;

    Slot0Configs slot0 = elevator2Config.Slot0;
    slot0.kP = Constants.kElevator2Proportional;
    slot0.kI = Constants.kElevator2Integral;
    slot0.kD = Constants.kElevator2Derivative;
    slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kV = Constants.kElevator2VelocityFeedFoward;
    //slot0.kS = Constants.kElevator1StaticFeedFoward; // The value of s is approximately the number of volts needed to get the mechanism moving

    //Setting the config option that allows playing music on the motor during disabled.
    //elevator2Config.Audio.AllowMusicDurDisable = true;
 
    StatusCode climberStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      climberStatus = elevator2.getConfigurator().apply(elevator2Config);
      if (climberStatus.isOK()) break;
    }
    if (!climberStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + climberStatus.toString());
    }
    m_Elevator2.setPosition(0);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
