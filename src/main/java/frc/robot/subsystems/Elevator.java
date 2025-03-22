/* // Copyright (c) FIRST and other WPILib contributors.
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {
  // Creates a new Elevator.

  TalonFX m_Elevator1 = new TalonFX(13, "rio");
  TalonFX m_Elevator2 = new TalonFX(14, "rio");

  private final PositionVoltage ElevatorPositionRequest = new PositionVoltage(0).withSlot(0);
  private final DutyCycleOut ElevatorVoltageRequest = new DutyCycleOut(0);

  private double ElevatorDesiredPos = 0;


  public Elevator() {
    this.configureElevator1(m_Elevator1);
    this.configureElevator2(m_Elevator2);
  }
  //############################################## BEGIN WRITING CLASS FUNCTIONS ######################################################

  public void setElevatorDesiredPosition(double DesiredPosition) {
    m_Elevator1.setControl(ElevatorPositionRequest.withPosition(DesiredPosition));
    m_Elevator2.setControl(ElevatorPositionRequest.withPosition(DesiredPosition));
    ElevatorDesiredPos = DesiredPosition;
  }

  public double getElevatorCurrentPosition(){
    return m_Elevator1.getPosition().getValueAsDouble();
  }

  public void setClimberZeroOutput() {
    m_Elevator1.setControl(ElevatorVoltageRequest);
    m_Elevator2.setControl(ElevatorVoltageRequest);
  }


  public boolean getElevatorInPosition(){
    if ((Math.abs(m_Elevator1.getPosition().getValueAsDouble() - ElevatorDesiredPos) < Constants.kAlgaeIntakePositionTolerance)){
      return true;
    } else {
      return false;
    }
  }

  public Command setElevatorPositionCommand(double targetPosition){
    return this.runOnce(() -> setElevatorDesiredPosition(targetPosition));
  }



//######################################### Start OF ELEVATOR CONFIGURATION ######################################################
//######################################### Start OF ELEVATOR CONFIGURATION ######################################################
//######################################### Start OF ELEVATOR CONFIGURATION ###################################################### 

//############################################# ELEVATOR 1 CONFIGURATION #################################################### 

  public void configureElevator1(TalonFX elevator1){
    TalonFXConfiguration elevator1Config = new TalonFXConfiguration();

    elevator1Config.MotorOutput.Inverted = Constants.kElevator1Direction;
    elevator1Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;


    elevator1Config.CurrentLimits.SupplyCurrentLimit = Constants.kElevator1SupplyCurrentLimit;
    elevator1Config.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevator1Config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kElevator1VoltageClosedLoopRampPeriod;
    elevator1Config.Voltage.PeakForwardVoltage = Constants.kElevator1MaxForwardVoltage;
    elevator1Config.Voltage.PeakReverseVoltage = Constants.kElevator1MaxReverseVoltage;
    

    Slot0Configs slot0 = elevator1Config.Slot0;
    slot0.kP = Constants.kElevator1Proportional;
    slot0.kI = Constants.kElevator1Integral;
    slot0.kD = Constants.kElevator1Derivative;

    slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kV = Constants.kElevator1VelocityFeedForward;
    slot0.kG = Constants.kElevator1GravityFeedForward;
    slot0.kS = Constants.kElevator1StaticFeedForward;
 


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

//############################################# ELEVATOR 1 CONFIGURATION #################################################### 


  public void configureElevator2(TalonFX elevator2){
    TalonFXConfiguration elevator2Config = new TalonFXConfiguration();

    elevator2Config.MotorOutput.Inverted = Constants.kElevator2Direction;
    elevator2Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevator2Config.CurrentLimits.SupplyCurrentLimit = Constants.kElevator2SupplyCurrentLimit;
    elevator2Config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kElevator2VoltageClosedLoopRampPeriod;
    elevator2Config.Voltage.PeakForwardVoltage = Constants.kElevator2MaxForwardVoltage;
    elevator2Config.Voltage.PeakReverseVoltage = Constants.kElevator2MaxReverseVoltage;
    elevator2Config.CurrentLimits.SupplyCurrentLimitEnable = true;

    Slot0Configs slot0 = elevator2Config.Slot0;
    slot0.kP = Constants.kElevator2Proportional;
    slot0.kI = Constants.kElevator2Integral;
    slot0.kD = Constants.kElevator2Derivative;
    slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kV = Constants.kElevator2VelocityFeedForward;
    slot0.kG = Constants.kElevator2GravityFeedForward;
    slot0.kS = Constants.kElevator2StaticFeedForward; // The value of s is approximately the number of volts needed to get the mechanism moving

 
    StatusCode elevator2Status = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      elevator2Status = elevator2.getConfigurator().apply(elevator2Config);
      if (elevator2Status.isOK()) break;
    }
    if (!elevator2Status.isOK()) {
      System.out.println("Could not configure device. Error: " + elevator2Status.toString());
    }
    m_Elevator2.setPosition(0);
  }

//################################################# END OF ELEVATOR CONFIGURATION ######################################################
//################################################# END OF ELEVATOR CONFIGURATION ######################################################
//################################################# END OF ELEVATOR CONFIGURATION ######################################################



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  }
 */