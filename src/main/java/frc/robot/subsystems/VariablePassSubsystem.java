// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// The shooter intake subsystem is the portion of the shooter that is NOT responsible for ejecting notes at high velocities.
public class VariablePassSubsystem extends SubsystemBase {

  public double m_LimelightTargetHeading = 0;

  public VariablePassSubsystem() {

  }
  
  public void setLimelightTargetHeading(double limelightTargetHeading) {
    m_LimelightTargetHeading = limelightTargetHeading;
  }
  
  public double getLimelightTargetHeading() {
    return m_LimelightTargetHeading;
  }

  /**
   * Sets the speed of the shooter intake motor.
   * @param speed The desired speed of the shooter intake, in rotations per second.
   */

  /**
   * Extends the intake hinge servo to allow the intake to pivot.
   */
  
  @Override
  public void initSendable(SendableBuilder builder){
    
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
