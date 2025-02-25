// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.BeamBreak;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class CoralCheckPosition extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CoralIntake m_coralSubsystem;
  //private final Elevator m_elevatorSubsystem;
  private final CommandXboxController m_driverController;

  private int m_stateMachine = 1;
  private double m_ElevatorTargetPos;
  private double elevatorStartPos;

  Timer PosCheckTimer = new Timer();

  @SuppressWarnings("unused")
    private boolean m_isFinished = false;
  
  
    /**
     * Creates a new set-PowerCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public CoralCheckPosition(CoralIntake coralSubsystem, CommandXboxController driverController) {
      m_coralSubsystem = coralSubsystem;
      //m_elevatorSubsystem = elevatorSubsystem;
      m_driverController = driverController;
  
     
      // Use addRequirements() here to declare subsystem dependencies.
      //addRequirements(m_elevatorSubsystem);
      addRequirements(m_coralSubsystem);
      }
   
        // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      //double elevatorStartPos = m_elevatorSubsystem.getElevatorPosition();
      m_isFinished = false;
      //PosCheckTimer.reset();
      //PosCheckTimer.start();
      m_coralSubsystem.setcoralSpinPower(Constants.kCoralOutSpinSpeed);  
      //m_ElevatorTargetPos = elevatorStartPos-1;
      m_stateMachine = 1;
    }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*switch(m_stateMachine){
      case 1:
      //Make sure coral rotate and algae rotate are in correct positions then move elevator
        if (m_coralSubsystem.getCoralRotateInPosition()) {
          m_stateMachine += 1;
          m_elevatorSubsystem.setElevatorDesiredPosition(m_ElevatorTargetPos);
        }
      break;
      case 2:
        //Make sure elevator is in position then end command
      if ((PosCheckTimer.get() < 4.00) && (m_driverController.getRightTriggerAxis() >= 0.5)) {
        m_coralSubsystem.setcoralSpinPower(Constants.kCoralOutSpinSpeed);  
        m_stateMachine += 1;
      } else {
         m_elevatorSubsystem.setElevatorDesiredPosition(elevatorStartPos);
      }
      break;
      case 3:*/
        // comments
        m_isFinished = true;
      //}
  }
//          m_coralSubsystem.setcoralSpinPower(Constants.kCoralOutSpinSpeed);  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //m_elevatorSubsystem.setElevatorDesiredPosition(elevatorStartPos);
  m_coralSubsystem.setcoralSpinPower(Constants.kCoralStopSpinSpeed);
  m_isFinished=true;
}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
