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
public class CoralIntakePosition extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final  Elevator m_Elevator;
  private final AlgaeIntake m_AlgaeIntake;
  private final CoralIntake m_CoralIntake;

  @SuppressWarnings("unused")
  private boolean m_isFinished = false;
  private int m_stateMachine = 1;
  private double m_ElevatorTargetPos = 0;


  /**
   * Creates a new set-PowerCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
    public CoralIntakePosition(Elevator elevator, AlgaeIntake algaeIntake, CoralIntake coralIntake) {
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
    m_CoralIntake.setcoralRotate(Constants.kCoralRotateHomePos);
    m_AlgaeIntake.setAlgaeRotatePos(Constants.kAlgaeHomePos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(m_stateMachine){
      case 1:
      //Make sure coral rotate and algae rotate are in correct positions then move elevator
        if (m_CoralIntake.getCoralRotateInPosition()) {
          m_stateMachine += 1;
          m_Elevator.setElevatorDesiredPosition(m_ElevatorTargetPos);
        }
      break;
      case 2:
      //Make sure elevator is in position then end command
      if (m_Elevator.getElevatorInPosition()) {
        m_isFinished = true;
      }
        
      break;
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
