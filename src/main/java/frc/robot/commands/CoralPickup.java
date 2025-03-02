// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class CoralPickup extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CoralIntake m_coralSubsystem;
  private final Elevator m_elevatorSubsystem;

  @SuppressWarnings("unused")
    private boolean m_isFinished = false;
  
  
    /**
     * Creates a new set-PowerCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public CoralPickup(CoralIntake coralSubsystem, Elevator elevatorSubsystem) {
      m_coralSubsystem = coralSubsystem;
      m_elevatorSubsystem = elevatorSubsystem;
  
     
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(m_elevatorSubsystem);
      addRequirements(m_coralSubsystem);
      
          
    }
   
        // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      m_isFinished = false;

      m_coralSubsystem.setcoralRotate(Constants.kCoralRotateHomePos);  
      m_coralSubsystem.setcoralSpinPower(Constants.kCoralInSpinSpeed); 
      m_coralSubsystem.setL4CoralSpitMode(false); 
    }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coralSubsystem.setcoralSpinPower(Constants.kCoralHoldSpinSpeed);
    m_isFinished = true;
  
}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
