// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
//import frc.robot.RepetitiveLogic.CoordinateTargets;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
//import frc.robot.subsystems.CoralIntake;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/** An example command that uses an example subsystem. */
public class Drive extends Command {
  private final CommandSwerveDrivetrain m_Drivetrain;
  private final CommandXboxController m_Controller;
  //private final CoordinateTargets m_CoordinateCalculator = new CoordinateTargets();
  private final Elevator m_Elevator;
  //private final CoralIntake m_CoralIntake;

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  @SuppressWarnings("unused")
  private boolean m_isFinished = false;

  private String driveRequest = "";
  private boolean UsingLimelight = false;
  
  private final PIDController drivePID, strafePID;
  private double aprilTagID;
  private double lockedID = 0;
  private double xTarget = 0;
  private double yTarget = 0;
  private double rotationTarget = 0;
  private Pose2d RobotPosition;
  private double speedControl = 1;

  private double autoAlignFlip = -1;
  Optional<Alliance> allianceColor = DriverStation.getAlliance();

  private double strafeController;
  private double driveController;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
 
  private final SwerveRequest.FieldCentricFacingAngle autoAlign = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(MaxSpeed*0.05).withHeadingPID(8, 0, 0.01)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  //private final SwerveRequest.RobotCentric L4Score = new SwerveRequest.RobotCentric();

//############################################## CLASS INITIALIZATION ##################################################################
  public Drive(CommandSwerveDrivetrain drive, CommandXboxController controller, Elevator elevator/*, CoralIntake coralIntake/*, VariablePassSubsystem variable*/) {
    m_Drivetrain = drive;
    m_Controller = controller;
    m_Elevator = elevator;
    //m_CoralIntake = coralIntake;

    this.drivePID = new PIDController(2.3, 0, 0.01); // 1, 0, 0
    this.strafePID = new PIDController(2.3, 0, 0.01); // 1, 0, 0

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Drivetrain);
    
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isFinished = false;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!DriverStation.isAutonomousEnabled()) {
    SmartDashboard.putNumber("XTarget", xTarget);
    SmartDashboard.putNumber("YTarget", yTarget);
    SmartDashboard.putNumber("Locked ID", lockedID);
    SmartDashboard.putNumber("April Tag ID", aprilTagID);
    RobotPosition = m_Drivetrain.getState().Pose;
    SmartDashboard.putNumber("RobotXPosition", RobotPosition.getX());
    SmartDashboard.putNumber("RobotYPosition", RobotPosition.getY());
    SmartDashboard.putNumber("Robot Angle", RobotPosition.getRotation().getDegrees());
    
    if (m_Controller.rightStick().getAsBoolean()) {
      speedControl = 0.8;
    } else if (m_Controller.leftStick().getAsBoolean()) {
      speedControl = 1;
    }

    if (lockedID != 0) {
      int[] validIDs = {Math.toIntExact(Math.round(lockedID))};
      LimelightHelpers.SetFiducialIDFiltersOverride("limelight-lime", validIDs);
    } 


    aprilTagID = LimelightHelpers.getFiducialID("limelight-lime");
    if ((m_Controller.povLeft().getAsBoolean() && m_Controller.povRight().getAsBoolean()
    && m_Controller.povDownLeft().getAsBoolean() && m_Controller.povUpLeft().getAsBoolean()
    || m_Controller.povDownRight().getAsBoolean() || m_Controller.povUpRight().getAsBoolean())
    && (LimelightHelpers.getTV(Constants.kLimelightName) || UsingLimelight)) 
    {
      driveRequest = "limelightDrive";
      UsingLimelight = true;
    } else {
      lockedID = 0;
      driveRequest = "";
      if (UsingLimelight = true) {
        int[] validIDs = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight-lime", validIDs);
      }
      UsingLimelight = false;
    }

    switch(driveRequest) {
      case "limelightDrive":
        //###########################################BEGIN DETERMINING TARGET POINTS################################################
        //###########################################BEGIN DETERMINING TARGET POINTS################################################
        aprilTagID = LimelightHelpers.getFiducialID("limelight-lime");
        if (m_Controller.povRight().getAsBoolean() || m_Controller.povDownRight().getAsBoolean() || m_Controller.povUpRight().getAsBoolean()) {
          if (lockedID == 0) {
            if (aprilTagID == 6) {
              xTarget = Constants.kRed6R[0];
              yTarget = Constants.kRed6R[1];
              rotationTarget = Constants.kRed6R[2];
              lockedID = 6;
            } else if (aprilTagID == 7) {
              xTarget = Constants.kRed7R[0];
              yTarget = Constants.kRed7R[1];
              rotationTarget = Constants.kRed7R[2];
              lockedID = 7;
            } else if (aprilTagID == 8) {
              xTarget = Constants.kRed8R[0];
              yTarget = Constants.kRed8R[1];
              rotationTarget = Constants.kRed8R[2];
              lockedID = 8;
            } else if (aprilTagID == 9) {
              xTarget = Constants.kRed9R[0];
            yTarget = Constants.kRed9R[1];
              rotationTarget = Constants.kRed9R[2];
              lockedID = 9;
            } else if (aprilTagID == 10) {
              xTarget = Constants.kRed10R[0];
              yTarget = Constants.kRed10R[1];
              rotationTarget = Constants.kRed10R[2];
              lockedID = 10;
            } else if (aprilTagID == 11) {
              xTarget = Constants.kRed11R[0];
              yTarget = Constants.kRed11R[1];
              rotationTarget = Constants.kRed11R[2];
              lockedID = 11;
            } else if (aprilTagID == 17) {
              xTarget = (Constants.kBlue17R[0] - 0.125);
              yTarget = (Constants.kBlue17R[1] - 0.2165075);
              rotationTarget = Constants.kBlue17R[2];
              lockedID = 17;
            } else if (aprilTagID == 18) {
              xTarget = (Constants.kBlue18R[0] - 0.25);
              yTarget = Constants.kBlue18R[1];
              rotationTarget = Constants.kBlue18R[2];
              lockedID = 18;
            } else if (aprilTagID == 19) {
              xTarget = (Constants.kBlue19R[0] - 0.125);
              yTarget = (Constants.kBlue19R[1] + 0.2165075);
              rotationTarget = Constants.kBlue19R[2];
              lockedID = 19;
            } else if (aprilTagID == 20) {
              xTarget = (Constants.kBlue20R[0] + 0.125);
              yTarget = (Constants.kBlue20R[1] + 0.2165075);
              rotationTarget = Constants.kBlue20R[2];
              lockedID = 20;
            } else if (aprilTagID == 21) {
              xTarget = (Constants.kBlue21R[0] + 0.25);
              yTarget = Constants.kBlue21R[1];
              rotationTarget = Constants.kBlue21R[2];
              lockedID = 21;
            } else if (aprilTagID == 22) {
              xTarget = (Constants.kBlue22R[0] + 0.125);
              yTarget = (Constants.kBlue22R[1] - 0.2165075);
              rotationTarget = Constants.kBlue22R[2];
              lockedID = 22;
            }
          } 
          if ((Math.abs(RobotPosition.getX() - xTarget) < 0.12) && (Math.abs(RobotPosition.getY() - yTarget) < 0.12)) {
            if (lockedID == 6) {
              xTarget = Constants.kRed6R[0];
              yTarget = Constants.kRed6R[1];
            } else if (lockedID == 7) {
              xTarget = Constants.kRed7R[0];
              yTarget = Constants.kRed7R[1];
            } else if (lockedID == 8) {
              xTarget = Constants.kRed8R[0];
              yTarget = Constants.kRed8R[1];
            } else if (lockedID == 9) {
              xTarget = Constants.kRed9R[0];
              yTarget = Constants.kRed9R[1];
            } else if (lockedID == 10) {
              xTarget = Constants.kRed10R[0];
              yTarget = Constants.kRed10R[1];
            } else if (lockedID == 11) {
              xTarget = Constants.kRed11R[0];
              yTarget = Constants.kRed11R[1];
            } else if (lockedID == 17) {
              xTarget = Constants.kBlue17R[0];
              yTarget = Constants.kBlue17R[1];
            } else if (lockedID == 18) {
              xTarget = Constants.kBlue18R[0];
              yTarget = Constants.kBlue18R[1];
            } else if (lockedID == 19) {
              xTarget = Constants.kBlue19R[0];
              yTarget = Constants.kBlue19R[1];
            } else if (lockedID == 20) {
              xTarget = Constants.kBlue20R[0];
              yTarget = Constants.kBlue20R[1];
            } else if (lockedID == 21) {
              xTarget = Constants.kBlue21R[0];
              yTarget = Constants.kBlue21R[1];
            } else if (lockedID == 22) {
              xTarget = Constants.kBlue22R[0];
              yTarget = Constants.kBlue22R[1];
            }
          }
        } else if ((m_Controller.povLeft().getAsBoolean() || m_Controller.povDownLeft().getAsBoolean()) || m_Controller.povUpLeft().getAsBoolean()) {
          if (lockedID == 0) {
            if (aprilTagID == 6) {
              xTarget = Constants.kRed6L[0];
              yTarget = Constants.kRed6L[1];
              rotationTarget = Constants.kRed6L[2];
              lockedID = 6;
            } else if (aprilTagID == 7) {
              xTarget = Constants.kRed7L[0];
              yTarget = Constants.kRed7L[1];
              rotationTarget = Constants.kRed7L[2];
              lockedID = 7;
            } else if (aprilTagID == 8) {
              xTarget = Constants.kRed8L[0];
              yTarget = Constants.kRed8L[1];
              rotationTarget = Constants.kRed8L[2];
              lockedID = 8;
            } else if (aprilTagID == 9) {
              xTarget = Constants.kRed9L[0];
              yTarget = Constants.kRed9L[1];
              rotationTarget = Constants.kRed9L[2];
              lockedID = 9;
            } else if (aprilTagID == 10) {
              xTarget = Constants.kRed10L[0];
              yTarget = Constants.kRed10L[1];
              rotationTarget = Constants.kRed10L[2];
              lockedID = 10;
            } else if (aprilTagID == 11) {
              xTarget = Constants.kRed11L[0];
              yTarget = Constants.kRed11L[1];
              rotationTarget = Constants.kRed11L[2];
              lockedID = 11;
            } else if (aprilTagID == 17) {
              xTarget = (Constants.kBlue17L[0] - 0.125);
              yTarget = (Constants.kBlue17L[1] - 0.2165075);
              rotationTarget = Constants.kBlue17L[2];
              lockedID = 17;
            } else if (aprilTagID == 18) {
              xTarget = (Constants.kBlue18L[0] - 0.25);
              yTarget = Constants.kBlue18L[1];
              rotationTarget = Constants.kBlue18L[2];
              lockedID = 18;
            } else if (aprilTagID == 19) {
              xTarget = (Constants.kBlue19L[0] - 0.125);
              yTarget = (Constants.kBlue19L[1] + 0.2165075);
              rotationTarget = Constants.kBlue19L[2];
              lockedID = 19;
            } else if (aprilTagID == 20) {
              xTarget = (Constants.kBlue20L[0] + 0.125);
              yTarget = (Constants.kBlue20L[1] + 0.2165075);
              rotationTarget = Constants.kBlue20L[2];
              lockedID = 20;
            } else if (aprilTagID == 21) {
              xTarget = (Constants.kBlue21L[0] + 0.25);
              yTarget = Constants.kBlue21L[1];
              rotationTarget = Constants.kBlue21L[2];
              lockedID = 21;
            } else if (aprilTagID == 22) {
              xTarget = (Constants.kBlue22L[0] + 0.125);
              yTarget = (Constants.kBlue22L[1] - 0.2165075);
              rotationTarget = Constants.kBlue22L[2];
              lockedID = 22;
            }
          }
          if (((Math.abs(RobotPosition.getX() - xTarget) < 0.12) && (Math.abs(RobotPosition.getY() - yTarget) < 0.12))) {
            if (lockedID == 6) {
              xTarget = Constants.kRed6L[0];
              yTarget = Constants.kRed6L[1];
            } else if (lockedID == 7) {
              xTarget = Constants.kRed7L[0];
              yTarget = Constants.kRed7L[1];
            } else if (lockedID == 8) {
              xTarget = Constants.kRed8L[0];
              yTarget = Constants.kRed8L[1];
            } else if (lockedID == 9) {
              xTarget = Constants.kRed9L[0];
              yTarget = Constants.kRed9L[1];
            } else if (lockedID == 10) {
              xTarget = Constants.kRed10L[0];
              yTarget = Constants.kRed10L[1];
            } else if (lockedID == 11) {
              xTarget = Constants.kRed11L[0];
              yTarget = Constants.kRed11L[1];
            } else if (lockedID == 17) {
              xTarget = Constants.kBlue17L[0];
              yTarget = Constants.kBlue17L[1];
            } else if (lockedID == 18) {
              xTarget = Constants.kBlue18L[0];
              yTarget = Constants.kBlue18L[1];
            } else if (lockedID == 19) {
              xTarget = Constants.kBlue19L[0];
              yTarget = Constants.kBlue19L[1];
            } else if (lockedID == 20) {
              xTarget = Constants.kBlue20L[0];
              yTarget = Constants.kBlue20L[1];
            } else if (lockedID == 21) {
              xTarget = Constants.kBlue21L[0];
              yTarget = Constants.kBlue21L[1];
            } else if (lockedID == 22) {
              xTarget = Constants.kBlue22L[0];
              yTarget = Constants.kBlue22L[1];
            }
        /*if (m_Controller.povRight().getAsBoolean()) {

           if (m_Elevator.getAtIntakingPos()) {
            targetPose2d = m_CoordinateCalculator.determineCoordinates(0, 0, true, true);
            xTarget = targetPose2d.getX();
            yTarget = targetPose2d.getY();
            rotationTarget = targetPose2d.getRotation().getDegrees();
          } else { 
          if (lockedID == 0) {
            targetPose2d = m_CoordinateCalculator.determineCoordinates(aprilTagID, 0, false, true);
            xTarget = targetPose2d.getX();
            yTarget = targetPose2d.getY();
            rotationTarget = targetPose2d.getRotation().getDegrees();
            lockedID = aprilTagID;
          } 
          if ((Math.abs(RobotPosition.getX() - xTarget) < 0.1) && (Math.abs(RobotPosition.getY() - yTarget) < 0.1)) {
            targetPose2d = m_CoordinateCalculator.determineCoordinates(aprilTagID, 1, false, true);
            xTarget = targetPose2d.getX();
            yTarget = targetPose2d.getY();
            rotationTarget = targetPose2d.getRotation().getDegrees();
          }
          //}
        } else if (m_Controller.povLeft().getAsBoolean()) {
          /* if (m_Elevator.getAtIntakingPos()) {
            targetPose2d = m_CoordinateCalculator.determineCoordinates(0, 0, true, false);
            xTarget = targetPose2d.getX();
            yTarget = targetPose2d.getY();
            rotationTarget = targetPose2d.getRotation().getDegrees();
          } else { 
          if (lockedID == 0) {
            targetPose2d = m_CoordinateCalculator.determineCoordinates(aprilTagID, 0, false, false);
            xTarget = targetPose2d.getX();
            yTarget = targetPose2d.getY();
            rotationTarget = targetPose2d.getRotation().getDegrees();
            lockedID = aprilTagID;
          }
          if (((Math.abs(RobotPosition.getX() - xTarget) < 0.1) && (Math.abs(RobotPosition.getY() - yTarget) < 0.1))) {
            targetPose2d = m_CoordinateCalculator.determineCoordinates(aprilTagID, 1, false, false);
            xTarget = targetPose2d.getX();
            yTarget = targetPose2d.getY();
            rotationTarget = targetPose2d.getRotation().getDegrees();
          //}
          }*/}
        } 

        //###########################################END OF DETERMINING TARGET POINTS################################################
        //###########################################END OF DETERMINING TARGET POINTS################################################

        strafeController = strafePID.calculate(RobotPosition.getY(), yTarget);
        driveController = drivePID.calculate(RobotPosition.getX(), xTarget);

        if (driveController > 1) {
          driveController = 1;
         } else if (driveController < -1) {
          driveController = -1;
         }
        if (strafeController > 1) {
          strafeController = 1;
        } else if (strafeController < -1) {
          strafeController = -1;
        }

        allianceColor = DriverStation.getAlliance();
        if (allianceColor.get() == Alliance.Red) {
          autoAlignFlip = -1;
        } else {
          autoAlignFlip = 1;
        }

        m_Drivetrain.setControl(autoAlign.withVelocityX(driveController * MaxSpeed * 0.4 * autoAlignFlip) // Drive forward with                                                                    
         .withVelocityY(strafeController * MaxSpeed * 0.4 * autoAlignFlip) // Drive left with negative X (left)
         .withTargetDirection(new Rotation2d(Math.toRadians(rotationTarget))));
      break;
      /*case "SpeedTest":
        m_Drivetrain.setControl(autoAlign.withVol);
      break;*/
      default:

      m_Drivetrain.setControl(drive.withVelocityX(-m_Controller.getLeftY() * MaxSpeed * m_Elevator.speedLimit * speedControl) // Drive forward with
      // negative Y (forward)
        .withVelocityY(-m_Controller.getLeftX() * MaxSpeed * m_Elevator.speedLimit * speedControl) // Drive left with negative X (left)
        .withRotationalRate(-m_Controller.getRightX() * MaxAngularRate * 1.3
        )); // Drive counterclockwise with negative X (left)
    }
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
