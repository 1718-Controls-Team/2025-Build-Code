// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CoralIntake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class CoralSpit extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CoralIntake m_coralSubsystem;

  private boolean m_isFinished = false;
  
    /**
     * Creates a new set-PowerCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public CoralSpit(CoralIntake coralSubsystem) {
      m_coralSubsystem = coralSubsystem;
  
      addRequirements(m_coralSubsystem);
    }
   
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      m_coralSubsystem.setcoralSpinPower(Constants.kCoralOutSpinSpeed);  
      m_coralSubsystem.setSpitting(true);
    }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_coralSubsystem.getL4CoralSpitMode() == true) {
      m_coralSubsystem.setcoralRotate(m_coralSubsystem.getCoralRotatePosition() - 0.6);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //m_coralSubsystem.setcoralSpinPower(Constants.kCoralStopSpinSpeed);
  m_coralSubsystem.setSpitting(false);
  
}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
