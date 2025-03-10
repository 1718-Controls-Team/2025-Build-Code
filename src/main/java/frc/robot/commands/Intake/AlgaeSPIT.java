// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntake;
import edu.wpi.first.wpilibj2.command.Command;


/** An example command that uses an example subsystem. */
public class AlgaeSPIT extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AlgaeIntake m_algaeSubsystem;

  
  private boolean m_isFinished = false;

  /**
   * Creates a new set-PowerCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlgaeSPIT(AlgaeIntake algaeSubsystem) {
    m_algaeSubsystem = algaeSubsystem;
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_algaeSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_isFinished = false;
      m_algaeSubsystem.setAlgaeOutput(-1);  
    }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_algaeSubsystem.setAlgaeSpinPower(Constants.kAlgaeStopSpinSpeed);  
    m_isFinished=true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
