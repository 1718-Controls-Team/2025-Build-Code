// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  TalonFX Elevator1 = new TalonFX(10);
  TalonFX Elevator2 = new TalonFX(10);

  DutyCycleOut energy = new DutyCycleOut(0);

  private final VelocityVoltage ElevatorVelocityRequest = new VelocityVoltage(0);

  public Elevator() {
    this.ElevatorMotorConfiguration(Elevator1);
    this.ElevatorMotorConfiguration(Elevator2);
  }

  public void ElevatorMotorConfiguration(TalonFX Elevator1) {

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
 
  public void setElevatorPower(double Erika) {
    Elevator1.setControl(ElevatorVelocityRequest.withVelocity(Erika));
    Elevator2.setControl(ElevatorVelocityRequest.withVelocity(Erika));
    System.out.println(Erika);
    
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
