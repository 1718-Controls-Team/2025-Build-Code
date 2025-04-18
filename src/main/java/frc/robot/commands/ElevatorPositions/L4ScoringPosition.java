// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorPositions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;


/** An example command that uses an example subsystem. */
public class L4ScoringPosition extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final  Elevator m_Elevator;
  private final AlgaeIntake m_AlgaeIntake;
  private final CoralIntake m_CoralIntake;

  
  private boolean m_isFinished = false;


  /**
   * Creates a new set-PowerCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
    public L4ScoringPosition(Elevator elevator, AlgaeIntake algaeIntake, CoralIntake coralIntake) {
    m_Elevator = elevator;
    m_AlgaeIntake = algaeIntake;
    m_CoralIntake = coralIntake;;
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Elevator);
    addRequirements(m_AlgaeIntake);
    addRequirements(m_CoralIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isFinished = false;
    m_CoralIntake.setcoralRotate(3);
    m_AlgaeIntake.setAlgaeRotatePos(Constants.kAlgaeHomePos);
    m_Elevator.setElevatorDesiredPosition(Constants.kElevatorL4ScoringPos);
    
    m_CoralIntake.setL4CoralSpitMode(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Elevator.getElevatorInPosition()) {
      m_CoralIntake.setcoralRotate(Constants.kCoralRotateL4Pos);
      m_isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
