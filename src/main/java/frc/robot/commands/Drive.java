// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VariablePassSubsystem;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/** An example command that uses an example subsystem. */
public class Drive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CommandSwerveDrivetrain m_Drivetrain;
  private final VariablePassSubsystem m_VariablePass;
  private final CommandXboxController m_Controller;

  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(null, null, null, null);

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private String driveRequest = "";
  private int m_LimelightStateMachine = 1;
  private boolean m_isFinished = false;
  private double m_AngleToAprilTag = 0;
  private double m_CurrentRobotHeading;
  private double m_NewAngleHeading;
  private double limeLightController = 0;
  private boolean UsingLimelight = false;
  private boolean doRejectUpdate = true;
  private final PIDController drivePID, strafePID, aimPID;
  private double aprilTagID;
  private double xTarget = 0;
  private double yTarget = 0;
  private double rotationTarget = 0;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1)
    .withRotationalDeadband(0.1)
    .withDriveRequestType(DriveRequestType.Velocity);

  
  /**
   * Creates a new set-PowerCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Drive(CommandSwerveDrivetrain drive, CommandXboxController controller, VariablePassSubsystem variable) {
    m_Drivetrain = drive;
    m_Controller = controller;
    m_VariablePass = variable;

    this.drivePID = new PIDController(0.058, 0, 0.0013); // 0.055, 0, 0.0013
    this.strafePID = new PIDController(0.058, 0, 0.0013); // 0.055, 0, 0.0013
    this.aimPID = new PIDController(0.058, 0, 0.0013); // 0.055, 0, 0.0013

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Drivetrain);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveFacingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    driveFacingAngle.HeadingController.setPID(2, 0, 0.1);

    m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((m_Controller.povLeft().getAsBoolean() || m_Controller.povRight().getAsBoolean()) && (LimelightHelpers.getTV(Constants.kLimelightName) || UsingLimelight)) {
      driveRequest = "limelightDrive";
      UsingLimelight = true;
      m_AngleToAprilTag = LimelightHelpers.getTX(Constants.kLimelightName);
      m_CurrentRobotHeading = m_Drivetrain.getPigeon2().getRotation3d().getAngle();
      m_NewAngleHeading = m_AngleToAprilTag + m_CurrentRobotHeading;
      m_VariablePass.setLimelightTargetHeading(m_NewAngleHeading);
      //m_RotationTarget = Rotation2d.fromDegrees(m_NewAngleHeading);
      SmartDashboard.putNumber("LIMELIGHT TX", m_AngleToAprilTag);
      SmartDashboard.putNumber("ROBOT HEADING (Pigeon)", m_CurrentRobotHeading);
    } else {
      driveRequest = "";
      UsingLimelight = false;
    }

    switch(driveRequest) {
      case "limeLightTurn":
        limeLightController = aimPID.calculate(m_Drivetrain.getPigeon2().getRotation3d().getAngle(), m_NewAngleHeading);
        if (limeLightController > 1) {
          limeLightController = 1;
         } else if (limeLightController < -1) {
          limeLightController = -1;
         }
         
        m_Drivetrain.setControl(drive.withVelocityX(-m_Controller.getLeftY() * MaxSpeed) // Drive forward with                                                                    
         .withVelocityY(-m_Controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
         .withRotationalRate(-limeLightController * MaxAngularRate)); // Drive counterclockwise with negative X (left)
         break;
      case "limelightDrive":
        aprilTagID = LimelightHelpers.getFiducialID("limelight-lime");
         if (m_Controller.povRight().getAsBoolean()) {
          if (aprilTagID == 6) {
            xTarget = Constants.kRedBottomRR[0];
            yTarget = Constants.kRedBottomRR[1];
            rotationTarget = Constants.kRedBottomRR[2];
          } else if (aprilTagID == 7) {
            xTarget = Constants.kRedRightR[0];
            yTarget = Constants.kRedRightR[1];
            rotationTarget = Constants.kRedRightR[2];
          } else if (aprilTagID == 8) {
            xTarget = Constants.kRedTopRR[0];
            yTarget = Constants.kRedTopRR[1];
            rotationTarget = Constants.kRedTopRR[2];
          } else if (aprilTagID == 9) {
            xTarget = Constants.kRedTopLR[0];
            yTarget = Constants.kRedTopLR[1];
            rotationTarget = Constants.kRedTopLR[2];
          } else if (aprilTagID == 10) {
            xTarget = Constants.kRedLeftR[0];
            yTarget = Constants.kRedLeftR[1];
            rotationTarget = Constants.kRedLeftR[2];
          } else if (aprilTagID == 11) {
            xTarget = Constants.kRedBottomLR[0];
            yTarget = Constants.kRedBottomLR[1];
            rotationTarget = Constants.kRedBottomLR[2];
          } else if (aprilTagID == 17) {
            xTarget = Constants.kBlueBottomLR[0];
            yTarget = Constants.kBlueBottomLR[1];
            rotationTarget = Constants.kBlueBottomLR[2];
          } else if (aprilTagID == 18) {
            xTarget = Constants.kBlueLeftR[0];
            yTarget = Constants.kBlueLeftR[1];
            rotationTarget = Constants.kBlueLeftR[2];
          } else if (aprilTagID == 19) {
            xTarget = Constants.kBlueTopLR[0];
            yTarget = Constants.kBlueTopLR[1];
            rotationTarget = Constants.kBlueTopLR[2];
          } else if (aprilTagID == 20) {
            xTarget = Constants.kBlueTopRR[0];
            yTarget = Constants.kBlueTopRR[1];
            rotationTarget = Constants.kBlueTopRR[2];
          } else if (aprilTagID == 21) {
            xTarget = Constants.kBlueRightR[0];
            yTarget = Constants.kBlueRightR[1];
            rotationTarget = Constants.kBlueRightR[2];
          } else if (aprilTagID == 22) {
            xTarget = Constants.kBlueBottomRR[0];
            yTarget = Constants.kBlueBottomRR[1];
            rotationTarget = Constants.kBlueBottomRR[2];
          }
         } else if (m_Controller.povLeft().getAsBoolean()) {
          if (aprilTagID == 6) {
            xTarget = Constants.kRedBottomRL[0];
            yTarget = Constants.kRedBottomRL[1];
            rotationTarget = Constants.kRedBottomRL[2];
          } else if (aprilTagID == 7) {
            xTarget = Constants.kRedRightL[0];
            yTarget = Constants.kRedRightL[1];
            rotationTarget = Constants.kRedRightL[2];
          } else if (aprilTagID == 8) {
            xTarget = Constants.kRedTopRL[0];
            yTarget = Constants.kRedTopRL[1];
            rotationTarget = Constants.kRedTopRL[2];
          } else if (aprilTagID == 9) {
            xTarget = Constants.kRedTopLL[0];
            yTarget = Constants.kRedTopLL[1];
            rotationTarget = Constants.kRedTopLL[2];
          } else if (aprilTagID == 10) {
            xTarget = Constants.kRedLeftL[0];
            yTarget = Constants.kRedLeftL[1];
            rotationTarget = Constants.kRedLeftL[2];
          } else if (aprilTagID == 11) {
            xTarget = Constants.kRedBottomLL[0];
            yTarget = Constants.kRedBottomLL[1];
            rotationTarget = Constants.kRedBottomLL[2];
          } else if (aprilTagID == 17) {
            xTarget = Constants.kBlueBottomLL[0];
            yTarget = Constants.kBlueBottomLL[1];
            rotationTarget = Constants.kBlueBottomLL[2];
          } else if (aprilTagID == 18) {
            xTarget = Constants.kBlueLeftL[0];
            yTarget = Constants.kBlueLeftL[1];
            rotationTarget = Constants.kBlueLeftL[2];
          } else if (aprilTagID == 19) {
            xTarget = Constants.kBlueTopLL[0];
            yTarget = Constants.kBlueTopLL[1];
            rotationTarget = Constants.kBlueTopLL[2];
          } else if (aprilTagID == 20) {
            xTarget = Constants.kBlueTopRL[0];
            yTarget = Constants.kBlueTopRL[1];
            rotationTarget = Constants.kBlueTopRL[2];
          } else if (aprilTagID == 21) {
            xTarget = Constants.kBlueRightL[0];
            yTarget = Constants.kBlueRightL[1];
            rotationTarget = Constants.kBlueRightL[2];
          } else if (aprilTagID == 22) {
            xTarget = Constants.kBlueBottomRL[0];
            yTarget = Constants.kBlueBottomRL[1];
            rotationTarget = Constants.kBlueBottomRL[2];
         }
        }
      break;
      default:
      m_Drivetrain.setControl(drive.withVelocityX(-m_Controller.getLeftY() * MaxSpeed) // Drive forward with
      // negative Y (forward)
        .withVelocityY(-m_Controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(-m_Controller.getRightX() * MaxAngularRate)); // Drive counterclockwise with negative X (left)
      break;
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
