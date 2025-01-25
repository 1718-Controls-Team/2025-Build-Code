// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BeamBreak extends SubsystemBase {
  /** Creates a new BeamBreak. */
  AnalogInput m_BeamBreakZeroAnalog = new AnalogInput(Constants.kBeamBreakZeroAnalog);
  AnalogInput m_BeamBreakOneAnalog = new AnalogInput(Constants.kBeamBreakOneAnalog);

  Debouncer m_IntakeDebounce = new Debouncer(0.05, Debouncer.DebounceType.kBoth);
  Debouncer m_ShooterDebounce = new Debouncer(0.05, Debouncer.DebounceType.kBoth);

  public BeamBreak() {
    m_BeamBreakZeroAnalog.setAverageBits(6);
    m_BeamBreakOneAnalog.setAverageBits(6);
    }

  /**
   * Example command factory method.
   *
   * @return a command
   */
 
    
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean getCurrentBeam() {  
    if (Constants.kPrintSubsystemBeamBreak){System.out.println("Subsystem: Move Motor " + m_BeamBreakOneAnalog.getAverageVoltage());}
    
    return (m_IntakeDebounce.calculate(m_BeamBreakZeroAnalog.getAverageVoltage() >= Constants.kIntakeBeamBreakCrossover));
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
