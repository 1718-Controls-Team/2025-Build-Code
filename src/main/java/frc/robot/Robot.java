// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command m_autonLoading;
  private final RobotContainer m_robotContainer;

  public boolean hasFilteredAutonRoutines = false;

  private final boolean kUseLimelight = true;

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_autonLoading = new PathPlannerAuto("Tests").ignoringDisable(true);
    m_autonLoading.schedule();

    //Setting up port forwarding for all limelight related ports.
    //Only setting the port-forwarding once in the code.
    for (int port = 5800; port <= 5807; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }

    //Set a custom brownout voltage for the RoboRIO.
    //Only works with the RIO2.
    RobotController.setBrownoutVoltage(Constants.kCustomBrownout);

    //Start a simple recording to the data log.
    //This should log the contents of the NetworkTables, which should be good for now.
    DataLogManager.start();
    //This should log the joysticks as well.
    DriverStation.startDataLog(DataLogManager.getLog());
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
    if (kUseLimelight) {
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
      
      LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if (llMeasurement != null && llMeasurement.tagCount > 0 && omegaRps < 2.0) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    //double m_DistanceBetweenAprilTagAndLimelight = Constants.kSpeakerAprilTagHeight - Constants.kLimelightHeight;
    SmartDashboard.putNumber("Pigeon", m_robotContainer.drivetrain.getPigeon2().getRotation2d().getDegrees());
    //double m_VerticalAngleToAprilTag = Math.toRadians(LimelightHelpers.getTY(Constants.kLimelightName));
    //double m_HorizontalAngleToAprilTag = Math.toRadians(LimelightHelpers.getTX(Constants.kLimelightName));
    //double m_DistanceToAprilTag = m_DistanceBetweenAprilTagAndLimelight / (Math.tan(m_VerticalAngleToAprilTag));
    //System.out.println(m_DistanceToAprilTag);
    //All of the commented stuff is calculations we did last year to find the distance from Limelight to April Tag

    //Check if the robot is in communication with the Driver Station.
    //If it is, attempt to filter the autonomous routines based on alliance color.
    
    /*if (!hasFilteredAutonRoutines && DriverStation.isDSAttached()) {
      switch (DriverStation.getAlliance().get()) {
        case Blue:
          m_robotContainer.autonSelect.filterSelections("Blue");
        break;
        case Red:
          m_robotContainer.autonSelect.filterSelections("Red");
        break;
      }
      m_robotContainer.autonSelect.sortSelections();
      hasFilteredAutonRoutines = true;*/
      
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = new WaitCommand(0.010).andThen(m_robotContainer.getAutonomousCommand());

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
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Pigeon", m_robotContainer.drivetrain.getPigeon2().getRotation2d().getDegrees());
  }

  @Override
  public void teleopExit() {}

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
