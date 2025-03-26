// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private Command m_autonLoading;
  private int m_auto2 = 0;


  boolean kUseLimelight = true;

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_autonLoading = new PathPlannerAuto("Tests").ignoringDisable(true);
    m_autonLoading.schedule();
    

    //Setting up port forwarding for all limelight related ports.
    //Only setting the port-forwarding once in the code.
    for (int port = 5800; port <= 5807; port++) {
      PortForwarder.add(port, "limelight-lime.local", port);
    }

    int[] validIDs = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 16, 17, 18, 19, 20, 21, 22};
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-lime", validIDs);

    //Set a custom brownout voltage for the RoboRIO.
    //Only works with the RIO2.
    RobotController.setBrownoutVoltage(Constants.kCustomBrownout);
    m_robotContainer.drivetrain.resetRotation(new Rotation2d(Math.PI));

    //Start a simple recording to the data log.
    //This should log the contents of the NetworkTables, which should be good for now.
    //DataLogManager.start();
    //This should log the joysticks as well.
    //DriverStation.startDataLog(DataLogManager.getLog());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();


    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */

     double headingDeg = m_robotContainer.drivetrain.getPigeon2().getRotation2d().getDegrees();
      
     LimelightHelpers.SetRobotOrientation("limelight-lime", headingDeg, 0, 0, 0, 0, 0);

    if (kUseLimelight) {
      var driveState = m_robotContainer.drivetrain.getState();
      
      LimelightHelpers.PoseEstimate llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-lime");
      if (llMeasurement != null && llMeasurement.tagCount > 0) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
        m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
      }
    }
  }

  @Override
  public void disabledInit() {
    
  }

  @Override
  public void disabledPeriodic() {
    
    SmartDashboard.putNumber("Pigeon", m_robotContainer.drivetrain.getPigeon2().getRotation2d().getDegrees());
  }

  @Override
  public void disabledExit() {
    kUseLimelight = false;
  }

  @Override
  public void autonomousInit() {
    kUseLimelight = false;
    if (m_auto2 == 0) {
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();
      m_auto2 = 1;
    } else {
      m_autonomousCommand = m_robotContainer.getAutonomous2Command();
    }

    

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //CommandScheduler.getInstance().schedule(m_robotContainer.runInitializeCommand());
    kUseLimelight=true;
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Pigeon", m_robotContainer.drivetrain.getPigeon2().getRotation2d().getDegrees());
  }

  @Override
  public void teleopExit() {
    kUseLimelight=false;
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
