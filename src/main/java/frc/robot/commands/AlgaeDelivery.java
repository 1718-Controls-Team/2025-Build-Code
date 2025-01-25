// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.BeamBreakSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;


/** An example command that uses an example subsystem. */
public class AlgaeDelivery extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AlgaeIntake m_algaeSubsystem;
  private final Elevator m_elevatorSubsystem;
  private final BeamBreakSubsystem m_beamBreakSubsystem;

  @SuppressWarnings("unused")
  private boolean m_isFinished = false;

  /**
   * Creates a new set-PowerCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlgaeDelivery(AlgaeIntake algaeSubsystem, Elevator elevatorSubsystem, BeamBreakSubsystem beamBreakSubsystem) {
    m_algaeSubsystem = algaeSubsystem;
    m_elevatorSubsystem = elevatorSubsystem;
    m_beamBreakSubsystem = beamBreakSubsystem;
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_algaeSubsystem);
    addRequirements(m_elevatorSubsystem);
    addRequirements(m_beamBreakSubsystem);

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isFinished = false;

    m_algaeSubsystem.setAlgaeRotatePos(Constants.kAlgaeHomePos);  
    m_algaeSubsystem.setAlgaeSpinPower(Constants.kAlgaeOutSpinSpeed);  
    m_elevatorSubsystem.setElevatorDesiredPosition(Constants.kElevatorAlgaePos);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  // might need something about the subsystem here
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_algaeSubsystem.setAlgaeSpinPower(Constants.kAlgaeStopSpinSpeed);  
    m_elevatorSubsystem.setElevatorDesiredPosition(Constants.kElevatorHomePos);
    m_isFinished=true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
