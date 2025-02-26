
package frc.robot.subsystems;


import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VariablePassSubsystem extends SubsystemBase {

  public double m_LimelightTargetHeading = 0;

  public VariablePassSubsystem() {

  }
  
  public void setLimelightTargetHeading(double limelightTargetHeading) {
    m_LimelightTargetHeading = limelightTargetHeading;
  }
  
  public double getLimelightTargetHeading() {
    return m_LimelightTargetHeading;
  }

  /**
   * Sets the speed of the shooter intake motor.
   * @param speed The desired speed of the shooter intake, in rotations per second.
   */
  
  @Override
  public void initSendable(SendableBuilder builder){
    
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {

  }
}
