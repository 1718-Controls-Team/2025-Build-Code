// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.BeamBreak;
import frc.robot.subsystems.CoralIntake;
import edu.wpi.first.wpilibj2.command.Command;


/** An example command that uses an example subsystem. */
public class AlgaePickup extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AlgaeIntake m_algaeSubsystem;
  private final CoralIntake m_coralSubsystem;


  @SuppressWarnings("unused")
  private boolean m_isFinished = false;

  /**
   * Creates a new set-PowerCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlgaePickup(AlgaeIntake algaeSubsystem, CoralIntake coralSubsystem) {
    m_algaeSubsystem = algaeSubsystem;
    m_coralSubsystem = coralSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_algaeSubsystem);
    addRequirements(m_coralSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isFinished = false;
      m_coralSubsystem.setcoralRotate(Constants.kCoralRotateAlgaePos);
      m_algaeSubsystem.setAlgaeSpinPower(Constants.kAlgaeInSpinSpeed);
      m_algaeSubsystem.setAlgaeRotatePos(Constants.kAlgaeIntakePos);

    }
    // the above elevator might not be correct
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  // might need something about the subsystem here
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_algaeSubsystem.setAlgaeSpinPower(Constants.kAlgaeIdleSpinSpeed);  
    m_isFinished=true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
