// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RepetitiveLogic.CoordinateTargets;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
//import frc.robot.subsystems.CoralIntake;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/** An example command that uses an example subsystem. */
public class Drive extends Command {
  private final CommandSwerveDrivetrain m_Drivetrain;
  private final CommandXboxController m_Controller;
  private final CoordinateTargets m_CoordinateCalculator = new CoordinateTargets();
  private final Elevator m_Elevator;
  //private final CoralIntake m_CoralIntake;

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  @SuppressWarnings("unused")
  private boolean m_isFinished = false;

  private String driveRequest = "";
  private boolean UsingLimelight = false;
  
  private final PIDController drivePID, strafePID, aimPID;
  private double aprilTagID;
  private double lockedID = 0;
  private double xTarget = 0;
  private double yTarget = 0;
  private double rotationTarget = 0;
  private Pose2d RobotPosition;
  private double RobotAngle;
  private double speedControl = 1;

  private Pose2d targetPose2d;

  private double turnController;
  private double strafeController;
  private double driveController;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
 
  private final SwerveRequest.FieldCentricFacingAngle autoAlign = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(MaxSpeed*0.1).withHeadingPID(8, 0, 0.01)
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
    this.aimPID = new PIDController(0.024, 0, 0.0); // 0.008, 0, 0.0013

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
    SmartDashboard.putNumber("Rotation Target", rotationTarget);
    
    if (m_Controller.rightStick().getAsBoolean()) {
      speedControl = 0.8;
    } else if (m_Controller.leftStick().getAsBoolean()) {
      speedControl = 1;
    }

    aprilTagID = LimelightHelpers.getFiducialID("limelight-lime");
    if ((m_Controller.povLeft().getAsBoolean() || m_Controller.povRight().getAsBoolean()) && (LimelightHelpers.getTV(Constants.kLimelightName) || UsingLimelight)) {
      driveRequest = "limelightDrive";
      UsingLimelight = true;
    } /*else if ((m_CoralIntake.getL4CoralSpitMode() == true) && (m_CoralIntake.getSpitting())) {
      driveRequest = "L4Score";
    }*/ else {
      lockedID = 0;
      driveRequest = "";
      UsingLimelight = false;
    }

    switch(driveRequest) {
      case "limelightDrive":
        //###########################################BEGIN DETERMINING TARGET POINTS################################################
        //###########################################BEGIN DETERMINING TARGET POINTS################################################
        aprilTagID = LimelightHelpers.getFiducialID("limelight-lime");
        if (m_Controller.povRight().getAsBoolean()) {
          if (lockedID == 0) {
            if (aprilTagID == 6) {
              xTarget = Constants.kRedBottomRR[0];
              yTarget = Constants.kRedBottomRR[1];
              rotationTarget = Constants.kRedBottomRR[2];
              lockedID = 6;
            } else if (aprilTagID == 7) {
              xTarget = Constants.kRedRightR[0];
              yTarget = Constants.kRedRightR[1];
              rotationTarget = Constants.kRedRightR[2];
              lockedID = 7;
            } else if (aprilTagID == 8) {
              xTarget = Constants.kRedTopRR[0];
              yTarget = Constants.kRedTopRR[1];
              rotationTarget = Constants.kRedTopRR[2];
              lockedID = 8;
            } else if (aprilTagID == 9) {
              xTarget = Constants.kRedTopLR[0];
            yTarget = Constants.kRedTopLR[1];
              rotationTarget = Constants.kRedTopLR[2];
              lockedID = 9;
            } else if (aprilTagID == 10) {
              xTarget = Constants.kRedLeftR[0];
              yTarget = Constants.kRedLeftR[1];
              rotationTarget = Constants.kRedLeftR[2];
              lockedID = 10;
            } else if (aprilTagID == 11) {
              xTarget = Constants.kRedBottomLR[0];
              yTarget = Constants.kRedBottomLR[1];
              rotationTarget = Constants.kRedBottomLR[2];
              lockedID = 11;
            } else if (aprilTagID == 17) {
              xTarget = (Constants.kBlueBottomLR[0] - 0.125);
              yTarget = (Constants.kBlueBottomLR[1] - 0.2165075);
              rotationTarget = Constants.kBlueBottomLR[2];
              lockedID = 17;
            } else if (aprilTagID == 18) {
              xTarget = (Constants.kBlueLeftR[0] - 1);
              yTarget = Constants.kBlueLeftR[1];
              rotationTarget = Constants.kBlueLeftR[2];
              lockedID = 18;
            } else if (aprilTagID == 19) {
              xTarget = (Constants.kBlueTopLR[0] - 0.125);
              yTarget = (Constants.kBlueTopLR[1] + 0.2165075);
              rotationTarget = Constants.kBlueTopLR[2];
              lockedID = 19;
            } else if (aprilTagID == 20) {
              xTarget = (Constants.kBlueTopRR[0] + 0.125);
              yTarget = (Constants.kBlueTopRR[1] + 0.2165075);
              rotationTarget = Constants.kBlueTopRR[2];
              lockedID = 20;
            } else if (aprilTagID == 21) {
              xTarget = (Constants.kBlueRightR[0] + 1);
              yTarget = Constants.kBlueRightR[1];
              rotationTarget = Constants.kBlueRightR[2];
              lockedID = 21;
            } else if (aprilTagID == 22) {
              xTarget = (Constants.kBlueBottomRR[0] + 0.125);
              yTarget = (Constants.kBlueBottomRR[1] - 0.2165075);
              rotationTarget = Constants.kBlueBottomRR[2];
              lockedID = 22;
            }
          } 
          if ((Math.abs(RobotPosition.getX() - xTarget) < 0.12) && (Math.abs(RobotPosition.getY() - yTarget) < 0.12)) {
            if (lockedID == 6) {
              xTarget = Constants.kRedBottomRR[0];
              yTarget = Constants.kRedBottomRR[1];
            } else if (lockedID == 7) {
              xTarget = Constants.kRedRightR[0];
              yTarget = Constants.kRedRightR[1];
            } else if (lockedID == 8) {
              xTarget = Constants.kRedTopRR[0];
              yTarget = Constants.kRedTopRR[1];
            } else if (lockedID == 9) {
              xTarget = Constants.kRedTopLR[0];
              yTarget = Constants.kRedTopLR[1];
            } else if (lockedID == 10) {
              xTarget = Constants.kRedLeftR[0];
              yTarget = Constants.kRedLeftR[1];
            } else if (lockedID == 11) {
              xTarget = Constants.kRedBottomLR[0];
              yTarget = Constants.kRedBottomLR[1];
            } else if (lockedID == 17) {
              xTarget = Constants.kBlueBottomLR[0];
              yTarget = Constants.kBlueBottomLR[1];
            } else if (lockedID == 18) {
              xTarget = Constants.kBlueLeftR[0];
              yTarget = Constants.kBlueLeftR[1];
            } else if (lockedID == 19) {
              xTarget = Constants.kBlueTopLR[0];
              yTarget = Constants.kBlueTopLR[1];
            } else if (lockedID == 20) {
              xTarget = Constants.kBlueTopRR[0];
              yTarget = Constants.kBlueTopRR[1];
            } else if (lockedID == 21) {
              xTarget = Constants.kBlueRightR[0];
              yTarget = Constants.kBlueRightR[1];
            } else if (lockedID == 22) {
              xTarget = Constants.kBlueBottomRR[0];
              yTarget = Constants.kBlueBottomRR[1];
            }
          }
        } else if (m_Controller.povLeft().getAsBoolean()) {
          if (lockedID == 0) {
            if (aprilTagID == 6) {
              xTarget = Constants.kRedBottomRL[0];
              yTarget = Constants.kRedBottomRL[1];
              rotationTarget = Constants.kRedBottomRL[2];
              lockedID = 6;
            } else if (aprilTagID == 7) {
              xTarget = Constants.kRedRightL[0];
              yTarget = Constants.kRedRightL[1];
              rotationTarget = Constants.kRedRightL[2];
              lockedID = 7;
            } else if (aprilTagID == 8) {
              xTarget = Constants.kRedTopRL[0];
              yTarget = Constants.kRedTopRL[1];
              rotationTarget = Constants.kRedTopRL[2];
              lockedID = 8;
            } else if (aprilTagID == 9) {
              xTarget = Constants.kRedTopLL[0];
              yTarget = Constants.kRedTopLL[1];
              rotationTarget = Constants.kRedTopLL[2];
              lockedID = 9;
            } else if (aprilTagID == 10) {
              xTarget = Constants.kRedLeftL[0];
              yTarget = Constants.kRedLeftL[1];
              rotationTarget = Constants.kRedLeftL[2];
              lockedID = 10;
            } else if (aprilTagID == 11) {
              xTarget = Constants.kRedBottomLL[0];
              yTarget = Constants.kRedBottomLL[1];
              rotationTarget = Constants.kRedBottomLL[2];
              lockedID = 11;
            } else if (aprilTagID == 17) {
              xTarget = (Constants.kBlueBottomLL[0] - 0.125);
              yTarget = (Constants.kBlueBottomLL[1] - 0.2165075);
              rotationTarget = Constants.kBlueBottomLL[2];
              lockedID = 17;
            } else if (aprilTagID == 18) {
              xTarget = (Constants.kBlueLeftL[0] - 0.25);
              yTarget = Constants.kBlueLeftL[1];
              rotationTarget = Constants.kBlueLeftL[2];
              lockedID = 18;
            } else if (aprilTagID == 19) {
              xTarget = (Constants.kBlueTopLL[0] - 0.125);
              yTarget = (Constants.kBlueTopLL[1] + 0.2165075);
              rotationTarget = Constants.kBlueTopLL[2];
              lockedID = 19;
            } else if (aprilTagID == 20) {
              xTarget = (Constants.kBlueTopRL[0] + 0.125);
              yTarget = (Constants.kBlueTopRL[1] + 0.2165075);
              rotationTarget = Constants.kBlueTopRL[2];
              lockedID = 20;
            } else if (aprilTagID == 21) {
              xTarget = (Constants.kBlueRightL[0] + 0.25);
              yTarget = Constants.kBlueRightL[1];
              rotationTarget = Constants.kBlueRightL[2];
              lockedID = 21;
            } else if (aprilTagID == 22) {
              xTarget = (Constants.kBlueBottomRL[0] + 0.125);
              yTarget = (Constants.kBlueBottomRL[1] - 0.2165075);
              rotationTarget = Constants.kBlueBottomRL[2];
              lockedID = 22;
            }
          }
          if (((Math.abs(RobotPosition.getX() - xTarget) < 0.12) && (Math.abs(RobotPosition.getY() - yTarget) < 0.12))) {
            if (lockedID == 6) {
              xTarget = Constants.kRedBottomRL[0];
              yTarget = Constants.kRedBottomRL[1];
            } else if (lockedID == 7) {
              xTarget = Constants.kRedRightL[0];
              yTarget = Constants.kRedRightL[1];
            } else if (lockedID == 8) {
              xTarget = Constants.kRedTopRL[0];
              yTarget = Constants.kRedTopRL[1];
            } else if (lockedID == 9) {
              xTarget = Constants.kRedTopLL[0];
              yTarget = Constants.kRedTopLL[1];
            } else if (lockedID == 10) {
              xTarget = Constants.kRedLeftL[0];
              yTarget = Constants.kRedLeftL[1];
            } else if (lockedID == 11) {
              xTarget = Constants.kRedBottomLL[0];
              yTarget = Constants.kRedBottomLL[1];
            } else if (lockedID == 17) {
              xTarget = Constants.kBlueBottomLL[0];
              yTarget = Constants.kBlueBottomLL[1];
            } else if (lockedID == 18) {
              xTarget = Constants.kBlueLeftL[0];
              yTarget = Constants.kBlueLeftL[1];
            } else if (lockedID == 19) {
              xTarget = Constants.kBlueTopLL[0];
              yTarget = Constants.kBlueTopLL[1];
            } else if (lockedID == 20) {
              xTarget = Constants.kBlueTopRL[0];
              yTarget = Constants.kBlueTopRL[1];
            } else if (lockedID == 21) {
              xTarget = Constants.kBlueRightL[0];
              yTarget = Constants.kBlueRightL[1];
            } else if (lockedID == 22) {
              xTarget = Constants.kBlueBottomRL[0];
              yTarget = Constants.kBlueBottomRL[1];
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

        RobotAngle = (m_Drivetrain.getPigeon2().getRotation2d().getDegrees() + 180)%360;
        if (RobotAngle < 5) {
           RobotAngle = RobotAngle + 360;
        }

        turnController = aimPID.calculate(RobotAngle, rotationTarget);
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
        if (turnController > 1) {
          turnController = 1;
        } else if (turnController < -1) {
          turnController = -1;
        }

        m_Drivetrain.setControl(autoAlign.withVelocityX(driveController * MaxSpeed * 0.35) // Drive forward with                                                                    
         .withVelocityY(strafeController * MaxSpeed * 0.35) // Drive left with negative X (left)
         .withTargetDirection(new Rotation2d(Math.toRadians(rotationTarget))));
         //.withRotationalRate(-turnController * MaxAngularRate)); // Drive counterclockwise with negative X (left)
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
