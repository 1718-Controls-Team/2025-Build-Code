// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CoralIntake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class AutonCoralPickup extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CoralIntake m_coralSubsystem;

    private boolean m_isFinished = false;
    Timer spitTimer = new Timer();
  
    /**
     * Creates a new set-PowerCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutonCoralPickup(CoralIntake coralSubsystem) {
      m_coralSubsystem = coralSubsystem;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(m_coralSubsystem);
    }
   
        // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      m_isFinished = false;
      
      m_coralSubsystem.setcoralSpinPower(Constants.kCoralInSpinSpeed);  
      spitTimer.reset();
      spitTimer.start();
    }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((spitTimer.get() > 2)){
      m_isFinished=true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_coralSubsystem.setcoralSpinPower(Constants.kCoralStopSpinSpeed);
  
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
