// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralIntake;
import edu.wpi.first.wpilibj2.command.Command;


/** An example command that uses an example subsystem. */
public class AlgaeSPIT extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AlgaeIntake m_algaeSubsystem;
  private final Elevator m_elevator;
  private final CoralIntake m_coralIntake;

  
  private boolean m_isFinished = false;
  private int m_stateMachine = 1;

  /**
   * Creates a new set-PowerCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlgaeSPIT(AlgaeIntake algaeSubsystem, Elevator elevator, CoralIntake coralIntake) {
    m_algaeSubsystem = algaeSubsystem;
    m_elevator = elevator;
    m_coralIntake = coralIntake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_algaeSubsystem);
    addRequirements(m_elevator);
    addRequirements(m_coralIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_stateMachine = 1;
      m_isFinished = false;
    }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_stateMachine) {
      case 1:
         m_coralIntake.setcoralRotate(Constants.kCoralRotateAlgaePos);
         m_algaeSubsystem.setAlgaeRotatePos(Constants.kAlgaeNetPos);
         m_stateMachine = 2;
        break;
    
      case 2:
        if (m_coralIntake.getCoralRotateInPosition()) {
          m_elevator.setElevatorDesiredPosition(Constants.kElevatorL4ScoringPos);
            if (m_elevator.getElevatorCurrentPosition() > 25) {
              m_algaeSubsystem.setAlgaeOutput(-1);  
              m_stateMachine = 0;
            }   
        }
      break;
    }

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
