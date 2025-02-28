// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.TClimber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class ClimberActivate extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TClimber m_tClimber;
  

  /**
   * Creates a new set-PowerCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimberActivate(TClimber tClimber) {
    m_tClimber = tClimber;
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_tClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_tClimber.setTClimberPosition(Constants.kTClimberUpPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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
