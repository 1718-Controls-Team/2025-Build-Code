package frc.robot.commands;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.commands.Auton.AlignToReef;

public class TeleOpCoralRoutine extends SubsystemBase{

    private AlignToReef m_alignmentGenerator;
    private CommandXboxController m_driveController;
    private Pose2d targetPose;
    private Pose2d intakePose;
    private double lockedID;
    private boolean leftSideCoral = false;
    private boolean redSide = false;
    Optional<Alliance> allianceColor = DriverStation.getAlliance();
    private Command teleOpCycleCommand;

    public TeleOpCoralRoutine(AlignToReef alignmentGenerator, CommandXboxController driveController) {
        m_alignmentGenerator = alignmentGenerator;
        m_driveController = driveController;
    }

    public Command generateTeleOpCycle(boolean leftSide) {
        lockedID = LimelightHelpers.getFiducialID("limelight-lime");
        if (leftSide == true) {
          if (lockedID == 6) {
              targetPose = new Pose2d(Constants.kRed6L[0], Constants.kRed6L[1], new Rotation2d(Math.toRadians(Constants.kRed6L[2])));
          } else if (lockedID == 7) {
              targetPose = new Pose2d(Constants.kRed7L[0], Constants.kRed7L[1], new Rotation2d(Math.toRadians(Constants.kRed7L[2])));
          } else if (lockedID == 8) {
              targetPose = new Pose2d(Constants.kRed8L[0], Constants.kRed8L[1], new Rotation2d(Math.toRadians(Constants.kRed8L[2])));
          } else if (lockedID == 9) {
              targetPose = new Pose2d(Constants.kRed9L[0], Constants.kRed9L[1], new Rotation2d(Math.toRadians(Constants.kRed9L[2])));
          } else if (lockedID == 10) {
              targetPose = new Pose2d(Constants.kRed10L[0], Constants.kRed10L[1], new Rotation2d(Math.toRadians(Constants.kRed10L[2])));
          } else if (lockedID == 11) {
              targetPose = new Pose2d(Constants.kRed11L[0], Constants.kRed11L[1], new Rotation2d(Math.toRadians(Constants.kRed11L[2])));
          } else if (lockedID == 17) {
              targetPose = new Pose2d(Constants.kBlue17L[0], Constants.kBlue17L[1], new Rotation2d(Math.toRadians(Constants.kBlue17L[2])));
          } else if (lockedID == 18) {
              targetPose = new Pose2d(Constants.kBlue18L[0], Constants.kBlue18L[1], new Rotation2d(Math.toRadians(Constants.kBlue18L[2])));
          } else if (lockedID == 19) {
              targetPose = new Pose2d(Constants.kBlue19L[0], Constants.kBlue19L[1], new Rotation2d(Math.toRadians(Constants.kBlue19L[2])));
          } else if (lockedID == 20) {
              targetPose = new Pose2d(Constants.kBlue20L[0], Constants.kBlue20L[1], new Rotation2d(Math.toRadians(Constants.kBlue20L[2])));
          } else if (lockedID == 21) {
              targetPose = new Pose2d(Constants.kBlue21L[0], Constants.kBlue21L[1], new Rotation2d(Math.toRadians(Constants.kBlue21L[2])));
          } else if (lockedID == 22) {
              targetPose = new Pose2d(Constants.kBlue22L[0], Constants.kBlue22L[1], new Rotation2d(Math.toRadians(Constants.kBlue22L[2])));
          }
        } else {
            if (lockedID == 6) {
                targetPose = new Pose2d(Constants.kRed6R[0], Constants.kRed6R[1], new Rotation2d(Math.toRadians(Constants.kRed6R[2])));
            } else if (lockedID == 7) {
                targetPose = new Pose2d(Constants.kRed7R[0], Constants.kRed7R[1], new Rotation2d(Math.toRadians(Constants.kRed7R[2])));
            } else if (lockedID == 8) {
                targetPose = new Pose2d(Constants.kRed8R[0], Constants.kRed8R[1], new Rotation2d(Math.toRadians(Constants.kRed8R[2])));
            } else if (lockedID == 9) {
                targetPose = new Pose2d(Constants.kRed9R[0], Constants.kRed9R[1], new Rotation2d(Math.toRadians(Constants.kRed9R[2])));
            } else if (lockedID == 10) {
                targetPose = new Pose2d(Constants.kRed10R[0], Constants.kRed10R[1], new Rotation2d(Math.toRadians(Constants.kRed10R[2])));
            } else if (lockedID == 11) {
                targetPose = new Pose2d(Constants.kRed11R[0], Constants.kRed11R[1], new Rotation2d(Math.toRadians(Constants.kRed11R[2])));
            } else if (lockedID == 17) {
                targetPose = new Pose2d(Constants.kBlue17R[0], Constants.kBlue17R[1], new Rotation2d(Math.toRadians(Constants.kBlue17R[2])));
            } else if (lockedID == 18) {
                targetPose = new Pose2d(Constants.kBlue18R[0], Constants.kBlue18R[1], new Rotation2d(Math.toRadians(Constants.kBlue18R[2])));
            } else if (lockedID == 19) {
                targetPose = new Pose2d(Constants.kBlue19R[0], Constants.kBlue19R[1], new Rotation2d(Math.toRadians(Constants.kBlue19R[2])));
            } else if (lockedID == 20) {
                targetPose = new Pose2d(Constants.kBlue20R[0], Constants.kBlue20R[1], new Rotation2d(Math.toRadians(Constants.kBlue20R[2])));
            } else if (lockedID == 21) {
                targetPose = new Pose2d(Constants.kBlue21R[0], Constants.kBlue21R[1], new Rotation2d(Math.toRadians(Constants.kBlue21R[2])));
            } else if (lockedID == 22) {
                targetPose = new Pose2d(Constants.kBlue22R[0], Constants.kBlue22R[1], new Rotation2d(Math.toRadians(Constants.kBlue22R[2])));
            }
        }

        allianceColor = DriverStation.getAlliance();
        if (allianceColor.get() == Alliance.Red) {
          redSide = true;
        } else {
          redSide = false;
        }

        if (redSide == false) {
           if (leftSideCoral) {
                intakePose = new Pose2d(1.657, 7.363, new Rotation2d(Math.toRadians(126)));
            } else {
                intakePose = new Pose2d(1.700, 0.716, new Rotation2d(Math.toRadians(-126)));
            } 
        } else {
            if (leftSideCoral) {
                intakePose = new Pose2d(16.126, 0.834, new Rotation2d(Math.toRadians(-54)));
            } else {
                intakePose = new Pose2d(15.931, 7.337, new Rotation2d(Math.toRadians(54)));
            } 
        }
        
        teleOpCycleCommand = Commands.sequence(
            m_alignmentGenerator.generateCommand(targetPose), Commands.waitUntil(m_driveController.rightTrigger(0.5)),
            Commands.waitUntil(m_driveController.rightTrigger(0.5).negate()), m_alignmentGenerator.generateCommand(intakePose),
            Commands.waitUntil(m_driveController.leftTrigger()), Commands.waitUntil(m_driveController.leftTrigger().negate()),
            m_alignmentGenerator.generateCommand(targetPose), Commands.waitUntil(m_driveController.rightTrigger(0.5)),
            Commands.waitUntil(m_driveController.rightTrigger(0.5).negate()), m_alignmentGenerator.generateCommand(intakePose),
            Commands.waitUntil(m_driveController.leftTrigger()), Commands.waitUntil(m_driveController.leftTrigger().negate()),
            m_alignmentGenerator.generateCommand(targetPose), Commands.waitUntil(m_driveController.rightTrigger(0.5)),
            Commands.waitUntil(m_driveController.rightTrigger(0.5).negate()), m_alignmentGenerator.generateCommand(intakePose),
            Commands.waitUntil(m_driveController.leftTrigger()), Commands.waitUntil(m_driveController.leftTrigger().negate())
        );
        return teleOpCycleCommand;
    }

    public Command setLeftSideCoralCommand(boolean GoToLeft) {
        return this.runOnce(() -> setLeftSideCoral(GoToLeft));
    }

    public void setLeftSideCoral(boolean GoToLeft) {
        this.leftSideCoral = GoToLeft;
    }

    public void cancelTeleopCycle() {
        teleOpCycleCommand.cancel();
    }

    public Command cancelTeleopCycleCommand() {
        return this.runOnce(() -> cancelTeleopCycle());
    }
}

