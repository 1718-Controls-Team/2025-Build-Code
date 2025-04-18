// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.TClimber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class ClimberInitialize extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final  Elevator m_Elevator;
  private final AlgaeIntake m_AlgaeIntake;
  private final CoralIntake m_CoralIntake;
  private final TClimber m_Climber;

  private boolean m_isFinished = false;

  /**
   * Creates a new set-PowerCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimberInitialize(Elevator elevator, AlgaeIntake algaeIntake, CoralIntake coralIntake, TClimber climber) {
    m_Elevator = elevator;
    m_AlgaeIntake = algaeIntake;
    m_CoralIntake = coralIntake;
    m_Climber = climber;
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Elevator);
    addRequirements(m_AlgaeIntake);
    addRequirements(m_CoralIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_CoralIntake.setcoralRotate(Constants.kCoralClimbPos);
    m_AlgaeIntake.setAlgaeRotatePos(Constants.kAlgaeClimbPos);
    m_Elevator.setElevatorDesiredPosition(Constants.kElevatorClimbPos);
    m_Climber.setTClimberPosition(114);
    if (m_Elevator.speedLimit < 1.0) {
      m_Elevator.setSpeedLimit(1);
    } else {
      m_Elevator.setSpeedLimit(0.35);
    }
    

    m_isFinished = true;
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
    return m_isFinished;
  }
}
