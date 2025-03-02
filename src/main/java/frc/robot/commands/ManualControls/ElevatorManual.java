// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualControls;

import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/** An example command that uses an example subsystem. */
public class ElevatorManual extends Command {
  private final Elevator m_elevatorSubsystem;
  private final CommandXboxController m_operator;

  private boolean flag = true;

  /**
   * Creates a new set-PowerCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorManual(Elevator elevatorSubsystem, CommandXboxController operator) {
    m_elevatorSubsystem = elevatorSubsystem;
    m_operator = operator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevatorSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((m_operator.getLeftY() > 0.2) || (m_operator.getLeftY() < -0.2)) {
      flag = false;
      m_elevatorSubsystem.setElevatorDesiredPosition(m_elevatorSubsystem.getElevatorCurrentPosition() + (3 * -m_operator.getLeftY()));
    } else if (flag == false) {
      m_elevatorSubsystem.setElevatorDesiredPosition(m_elevatorSubsystem.getElevatorCurrentPosition());
      flag = true;
    }
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
