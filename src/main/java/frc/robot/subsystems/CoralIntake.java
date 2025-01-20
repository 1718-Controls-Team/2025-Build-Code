// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class CoralIntake extends SubsystemBase {
  /** Creates a new CoralIntake. */
  TalonFX coralIntake = new TalonFX(10);
  TalonFX coralRotate = new TalonFX(10);
  DutyCycleOut energy = new DutyCycleOut(0);

  private final VelocityVoltage coralMotorVelocityRequest = new VelocityVoltage(0);

  public CoralIntake() {
    this.coralIntakeConfiguration(coralIntake);
    this.coralIntakeConfiguration(coralRotate);

  }

  public void coralIntakeConfiguration(TalonFX coralIntake) {

  }

  /**
   * Example command factory method.
   *
   * @return a command 
   */
 
  public void setcoralMotorPower(double Erika) {
    coralIntake.setControl(coralMotorVelocityRequest.withVelocity(Erika));
    coralRotate.setControl(coralMotorVelocityRequest.withVelocity(Erika));
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
