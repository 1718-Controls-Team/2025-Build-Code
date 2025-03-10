// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;


/** An example command that uses an example subsystem. */
public class AutoL4Scoring extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final  Elevator m_Elevator;
  private final AlgaeIntake m_AlgaeIntake;
  private final CoralIntake m_coralSubsystem;

  private int m_stateMachine = 1;
  private boolean m_isFinished = false;
  Timer spitTimer = new Timer();


  /**
   * Creates a new set-PowerCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
    public AutoL4Scoring(Elevator elevator, AlgaeIntake algaeIntake, CoralIntake coralSubsystem) {
    m_Elevator = elevator;
    m_AlgaeIntake = algaeIntake;
    m_coralSubsystem = coralSubsystem;
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Elevator);
    addRequirements(m_AlgaeIntake);
    addRequirements(m_coralSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_coralSubsystem.setcoralRotate(Constants.kCoralRotateL4Pos);
    m_AlgaeIntake.setAlgaeRotatePos(Constants.kAlgaeHomePos);
    m_Elevator.setElevatorDesiredPosition(Constants.kElevatorL4ScoringPos);
    m_coralSubsystem.setL4CoralSpitMode(true);
    m_isFinished = false;
    m_stateMachine = 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_stateMachine) {
      case 1:
      m_coralSubsystem.setcoralSpinPower(Constants.kCoralOutSpinSpeed);  
      m_coralSubsystem.setSpitting(true);
      spitTimer.reset();
      spitTimer.start();
      m_stateMachine = 2;
        break;
      case 2:
      for(int i = 0; i < 5000; ++i) {
        m_coralSubsystem.setcoralRotate(m_coralSubsystem.getCoralRotatePosition() - 0.6);
        if (spitTimer.get() >= 2) break;
      }
      m_stateMachine = 3;
        break;
      case 3:
        m_coralSubsystem.setSpitting(false);
        m_coralSubsystem.setL4CoralSpitMode(false);
        m_coralSubsystem.setcoralSpinPower(Constants.kCoralStopSpinSpeed);  
        m_stateMachine = 0;
        m_isFinished = true;
      
        break;
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
