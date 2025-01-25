// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.BeamBreakSubsystem;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class CoralPickup extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CoralIntake m_coralSubsystem;
  private final Elevator m_elevatorSubsystem;
  private final BeamBreakSubsystem m_beamBreakSubsystem;

    private boolean m_isFinished = false;
  
  
    /**
     * Creates a new set-PowerCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public CoralPickup(CoralIntake coralSubsystem, Elevator elevatorSubsystem, BeamBreakSubsystem beamBreakSubsystem) {
      m_coralSubsystem = coralSubsystem;
      m_elevatorSubsystem = elevatorSubsystem;
      m_beamBreakSubsystem = beamBreakSubsystem;
  
     
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(m_elevatorSubsystem);
      addRequirements(m_coralSubsystem);
      
          
        }
   
        // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      m_isFinished = false;
    
    /** if (m_beamBreakSubsystem.getCoralPresent()){ //Check to see if we have a coral present, and end command if we do.
       m_isFinished=true;
       */
     //If we don't have a coral,  set speeds and move to execute
      m_coralSubsystem.setcoralRotate(Constants.kCoralIntakeDownPos);  
      m_coralSubsystem.setcoralSpinPower(Constants.kCoralIntakeSpeed);  
      //m_shooterSubsystem.setShooterSpeed(0); Temporarily disabling this for auto testing
      m_elevatorSubsystem.setElevatorDesiredPosition(Constants.kElevatorIntakePos);
     
    }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
        
      }

  
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  if (m_beamBreakSubsystem.getCoralPresentIntake())
    m_elevatorSubsystem.setElevatorPosition(Constants.kElevatorHomePos);
    m_coralSubsystem.setcoralSpinPower(Constants.kCoralSpinStopSpeed);
    m_isFinished=true;
  
}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
