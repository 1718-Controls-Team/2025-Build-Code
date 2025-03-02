// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualControls;

import frc.robot.subsystems.TClimber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/** An example command that uses an example subsystem. */
public class ClimberManual extends Command {
  private final TClimber m_climberSubsystem;
  private final CommandXboxController m_driver;

  private boolean flag = true;

  /**
   * Creates a new set-PowerCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimberManual(TClimber climberSubsystem, CommandXboxController driver) {
    m_climberSubsystem = climberSubsystem;
    m_driver = driver;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climberSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_driver.povUp().getAsBoolean()) {
      m_climberSubsystem.setTClimberPosition(m_climberSubsystem.getClimberCurrentPosition() + 10);
      flag = false;
    } else if (m_driver.povDown().getAsBoolean()) {
      m_climberSubsystem.setTClimberPosition(m_climberSubsystem.getClimberCurrentPosition() - 10);
    } else if (flag == false){
      m_climberSubsystem.setTClimberPosition(m_climberSubsystem.getClimberCurrentPosition());
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
