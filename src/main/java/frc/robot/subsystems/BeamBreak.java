// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// The shooter intake subsystem is the portion of the shooter that is NOT responsible for ejecting notes at high velocities.
public class BeamBreak extends SubsystemBase {

  //Open sensors
  AnalogInput m_BeamBreakZeroAnalog = new AnalogInput(Constants.kBeamBreakZeroAnalog);
  AnalogInput m_BeamBreakOneAnalog = new AnalogInput(Constants.kBeamBreakOneAnalog);

  Debouncer m_IntakeDebounce = new Debouncer(0.05, Debouncer.DebounceType.kBoth);
  Debouncer m_ShooterDebounce = new Debouncer(0.05, Debouncer.DebounceType.kBoth);

  //Open Servo
  //Servo intakeHinge = new Servo(Constants.kShooterIntakePivotReleasePWM);

  public BeamBreak() {
  m_BeamBreakZeroAnalog.setAverageBits(6);
  m_BeamBreakOneAnalog.setAverageBits(6);
  }
  
  // Start of sensor related methods
  /**
   * Check if a note is in front of the intake beam break.
   * @return Whether the intake beam break detects a note.
   * True or false.
   */
  public BooleanSupplier getCoralPresentIntake() {  
    if (Constants.kPrintSubsystemBeamBreak){System.out.println("Subsystem: Shooter - getNotePresentShooter Voltage " + m_BeamBreakOneAnalog.getAverageVoltage());}
    
    return (m_IntakeDebounce.calculate(m_BeamBreakZeroAnalog.getAverageVoltage() >= Constants.kIntakeBeamBreakCrossover));
  }
  
  /**
   * Check if a note is in front of the shooter beam break.
   * @return Whether the shooter beam break detects a note.
   * True or false.
   */
  public boolean getAlgaePresentIntake() {  
    if (Constants.kPrintSubsystemBeamBreak){
      System.out.println("Subsystem: Shooter - getNotePresentShooter Voltage " + m_BeamBreakOneAnalog.getAverageVoltage());
      System.out.println("Subsystem: Shooter - getNotePresentShooter :" + (m_BeamBreakOneAnalog.getAverageVoltage() >= Constants.kShooterBeamBreakCrossover));
    }
    return (m_ShooterDebounce.calculate(m_BeamBreakOneAnalog.getAverageVoltage() >= Constants.kShooterBeamBreakCrossover));
    
  }
  // End of sensor related methods

  /**
   * Check if a note is in front of either the intake beam break or the shooter beam break.
   * @return Whether either beam break detects a note.
   * True or false.
   */
  public boolean getElementPresent() {
    if (Constants.kPrintSubsystemBeamBreak){
      System.out.println("Subsystem: Shooter - getNotePresentShooter Voltage " + m_BeamBreakOneAnalog.getAverageVoltage());
      System.out.println("Subsystem: Shooter - getNotePresentIntake Voltage " + m_BeamBreakZeroAnalog.getAverageVoltage());
    }
    return (this.getCoralPresentIntake() || this.getAlgaePresentIntake());
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
    builder.setSmartDashboardType("BeamBreakSubsystem");
    builder.addBooleanProperty("Coral Present in Intake?", this::getCoralPresentIntake, null); 
    builder.addBooleanProperty("Algae Present in Intake?", this::getAlgaePresentIntake, null);
    builder.addDoubleProperty("Intake Beam Break Voltage", () -> {return m_BeamBreakZeroAnalog.getAverageVoltage();}, null);
    builder.addDoubleProperty("Shooter Beam Break Voltage", () -> {return m_BeamBreakOneAnalog.getAverageVoltage();}, null);
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
