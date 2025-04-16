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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private Command m_autonLoading;
  private int m_auto2 = 0;
  private Field2d field2d = new Field2d();
  private Field2d odomField = new Field2d();
  private boolean useMegatag2 = false;
  LimelightHelpers.PoseEstimate llMeasurementReef;
  LimelightHelpers.PoseEstimate llMeasurementCoral;

  boolean kUseLimelightReef = true;
  boolean kUseLimelightCoral = true;

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_autonLoading = new PathPlannerAuto("Tests").ignoringDisable(true);
    m_autonLoading.schedule();
    

    //Setting up port forwarding for all limelight related ports.
    //Only setting the port-forwarding once in the code.
    for (int port = 5800; port <= 5807; port++) {
      PortForwarder.add(port, "limelight-lime.local", port);
      PortForwarder.add(port, "limelight-coral.local", port);
    }

    int[] validIDs = {9, 10, 11, 20, 21, 22};
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-lime", validIDs);

    int[] coralIDs = {6, 8, 17, 19};
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-coral", coralIDs);

    //Set a custom brownout voltage for the RoboRIO.
    //Only works with the RIO2.
    RobotController.setBrownoutVoltage(Constants.kCustomBrownout);
    m_robotContainer.drivetrain.resetRotation(new Rotation2d(Math.PI));

    //Start a simple recording to the data log.
    //This should log the contents of the NetworkTables, which should be good for now.
    //DataLogManager.start();
    //This should log the joysticks as well.
    //DriverStation.startDataLog(DataLogManager.getLog());
    SmartDashboard.putData(field2d);
    SmartDashboard.putData(odomField);
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

    double headingDeg = m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
      
    LimelightHelpers.SetRobotOrientation("limelight-lime", headingDeg, 0, 0, 0, 0, 0);
    if (useMegatag2) {
      llMeasurementReef = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-lime");
    } else {
      llMeasurementReef = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-lime");
    }

    LimelightHelpers.SetRobotOrientation("limelight-coral", headingDeg, 0, 0, 0, 0, 0);
    if (kUseLimelightCoral && LimelightHelpers.getTV("limelight-coral")) {
      llMeasurementCoral = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-coral");
      
      if (llMeasurementCoral != null && llMeasurementCoral.tagCount > 0 && (m_robotContainer.drivetrain.getState().Speeds.omegaRadiansPerSecond < 1.8)) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurementCoral.pose, Utils.fpgaToCurrentTime(llMeasurementCoral.timestampSeconds),VecBuilder.fill(0.1, 0.1, .1));
      }
    }


    if (kUseLimelightReef && LimelightHelpers.getTV("limelight-lime")) {
      var driveState = m_robotContainer.drivetrain.getState();
      //LimelightHelpers.PoseEstimate llMeasurementReef = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-lime");
      
      field2d.setRobotPose(llMeasurementReef.pose);
      odomField.setRobotPose(driveState.Pose);
      if (llMeasurementReef != null && llMeasurementReef.tagCount > 0 && (m_robotContainer.drivetrain.getState().Speeds.omegaRadiansPerSecond < 1.5)) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurementReef.pose, Utils.fpgaToCurrentTime(llMeasurementReef.timestampSeconds),VecBuilder.fill(0.1, 0.1, .1));
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
    
  }

  @Override
  public void autonomousInit() {
    useMegatag2 = true;
    int[] validIDs = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-lime", validIDs);
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
  public void autonomousExit() {
    useMegatag2 = false;
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //CommandScheduler.getInstance().schedule(m_robotContainer.runInitializeCommand());
    int[] validIDs = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-lime", validIDs);
    kUseLimelightReef= true;
    useMegatag2 = true;
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Pigeon", m_robotContainer.drivetrain.getPigeon2().getRotation2d().getDegrees());
  }

  @Override
  public void teleopExit() {
    
    useMegatag2 = false;
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
