// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;


/** An example command that uses an example subsystem. */
public class MoveElevator extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final  Elevator m_Elevator;
  private final AlgaeIntake m_AlgaeIntake;
  private final CoralIntake m_CoralIntake;

  private boolean m_isFinished = false;
  private int m_stateMachine = 1;


  /**
   * Creates a new set-PowerCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
    public MoveElevator(Elevator elevator, AlgaeIntake algaeIntake, CoralIntake coralIntake) {
    m_Elevator = elevator;
    m_AlgaeIntake = algaeIntake;
    m_CoralIntake = coralIntake;
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Elevator);
    addRequirements(m_AlgaeIntake);
    addRequirements(m_CoralIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  // might need something about the subsystem here
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
