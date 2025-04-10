package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.commands.Auton.AlignToReef;

public class TeleOpCoralRoutine {

    private AlignToReef m_alignmentGenerator;
    private CommandXboxController m_controller;
    private Pose2d targetPose;
    private double lockedID;

    public TeleOpCoralRoutine(AlignToReef alignmentGenerator, CommandXboxController controller) {
        m_alignmentGenerator = alignmentGenerator;
        m_controller = controller;
    }

    public Command generateTeleOpCycle(Pose2d scorePose, Pose2d intakePose, boolean leftRight) {
        lockedID = LimelightHelpers.getFiducialID("limelight-lime");
        if (lockedID == 6) {
            targetPose = new Pose2d(Constants.kRed6L[0], Constants.kRed6L[1], new Rotation2d(Constants.kRed6L[2]));
        } else if (lockedID == 7) {
            targetPose = new Pose2d(Constants.kRed7L[0], Constants.kRed7L[1], new Rotation2d(Constants.kRed7L[2]));
        } else if (lockedID == 8) {
            targetPose = new Pose2d(Constants.kRed8L[0], Constants.kRed8L[1], new Rotation2d(Constants.kRed8L[2]));
        } else if (lockedID == 9) {
            targetPose = new Pose2d(Constants.kRed9L[0], Constants.kRed9L[1], new Rotation2d(Constants.kRed9L[2]));
        } else if (lockedID == 10) {
            targetPose = new Pose2d(Constants.kRed10L[0], Constants.kRed10L[1], new Rotation2d(Constants.kRed10L[2]));
        } else if (lockedID == 11) {
            targetPose = new Pose2d(Constants.kRed11L[0], Constants.kRed11L[1], new Rotation2d(Constants.kRed11L[2]));
        } else if (lockedID == 17) {
            targetPose = new Pose2d(Constants.kBlue17L[0], Constants.kBlue17L[1], new Rotation2d(Constants.kBlue17L[2]));
        } else if (lockedID == 18) {
            targetPose = new Pose2d(Constants.kBlue18L[0], Constants.kBlue18L[1], new Rotation2d(Constants.kBlue18L[2]));
        } else if (lockedID == 19) {
            targetPose = new Pose2d(Constants.kBlue19L[0], Constants.kBlue19L[1], new Rotation2d(Constants.kBlue19L[2]));
        } else if (lockedID == 20) {
            targetPose = new Pose2d(Constants.kBlue20L[0], Constants.kBlue20L[1], new Rotation2d(Constants.kBlue20L[2]));
        } else if (lockedID == 21) {
            targetPose = new Pose2d(Constants.kBlue21L[0], Constants.kBlue21L[1], new Rotation2d(Constants.kBlue21L[2]));
        } else if (lockedID == 22) {
            targetPose = new Pose2d(Constants.kBlue22L[0], Constants.kBlue22L[1], new Rotation2d(Constants.kBlue22L[2]));
        }

        return Commands.sequence(
            m_alignmentGenerator.generateCommand(scorePose), Commands.waitUntil(m_controller.rightTrigger(0.5)),
            Commands.waitUntil(m_controller.rightTrigger(0.5).negate()), m_alignmentGenerator.generateCommand(intakePose),
            Commands.waitUntil(m_controller.leftTrigger()), Commands.waitUntil(m_controller.leftTrigger().negate()),
            m_alignmentGenerator.generateCommand(scorePose), Commands.waitUntil(m_controller.rightTrigger(0.5)),
            Commands.waitUntil(m_controller.rightTrigger(0.5).negate()), m_alignmentGenerator.generateCommand(intakePose),
            Commands.waitUntil(m_controller.leftTrigger()), Commands.waitUntil(m_controller.leftTrigger().negate()),
            m_alignmentGenerator.generateCommand(scorePose), Commands.waitUntil(m_controller.rightTrigger(0.5)),
            Commands.waitUntil(m_controller.rightTrigger(0.5).negate()), m_alignmentGenerator.generateCommand(intakePose),
            Commands.waitUntil(m_controller.leftTrigger()), Commands.waitUntil(m_controller.leftTrigger().negate())
        );
    }

    public BooleanEvent controllerReleaseSpit() {
        
        return new BooleanEvent(null, null);
    }
}

