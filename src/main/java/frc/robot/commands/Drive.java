// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VariablePassSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/** An example command that uses an example subsystem. */
public class Drive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CommandSwerveDrivetrain m_Drivetrain;
  private final VariablePassSubsystem m_VariablePass;
  private final CommandXboxController m_Controller;

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private String driveRequest = "";
  private int m_LimelightStateMachine = 1;
  private boolean m_isFinished = false;
  private double m_AngleToAprilTag = 0;
  private double m_CurrentRobotHeading;
  private double m_NewAngleHeading;
  private PIDController aimPID = new PIDController(0.058, 0, 0.0013); // 0.055, 0, 0.0013
  private double limeLightController = 0;
  private boolean LimeLightShootingFlag = false;

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
    if ((m_Controller.povLeft().getAsBoolean() || m_Controller.povRight().getAsBoolean()) && (LimelightHelpers.getTV(Constants.kLimelightName) || LimeLightShootingFlag)) {
      driveRequest = "limeLightAim";
      LimeLightShootingFlag = true;
      m_AngleToAprilTag = LimelightHelpers.getTX(Constants.kLimelightName);
      m_CurrentRobotHeading = m_Drivetrain.getPigeon2().getRotation3d().getAngle();
      m_NewAngleHeading = m_AngleToAprilTag + m_CurrentRobotHeading;
      m_VariablePass.setLimelightTargetHeading(m_NewAngleHeading);
      //m_RotationTarget = Rotation2d.fromDegrees(m_NewAngleHeading);
      SmartDashboard.putNumber("LIMELIGHT TX", m_AngleToAprilTag);
      SmartDashboard.putNumber("ROBOT HEADING (Pigeon)", m_CurrentRobotHeading);
    } else {
      driveRequest = "";
      LimeLightShootingFlag = false;
    }

    switch(driveRequest) {
      case "limeLightAim":
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
