package frc.robot.commands.Auton;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutonPIDCommandTest extends Command{
    
    public CommandSwerveDrivetrain mSwerve;
    public final Pose2d goalPose;
    private PPHolonomicDriveController mDriveController = TunerConstants.holonomicControls;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric();
    private final SwerveRequest.FieldCentricFacingAngle forwardStraight = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(MaxSpeed*0.1).withHeadingPID(8, 0, 0.01)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final Trigger endTrigger;
    private final Trigger endTriggerDebounced;
    private final PIDController drivePID, strafePID;

    private LimelightHelpers.PoseEstimate LLPose;
    private double strafeController;
    private double driveController;
    private final Timer timer = new Timer();
  
    private final BooleanPublisher endTriggerLogger = NetworkTableInstance.getDefault().getTable("logging").getBooleanTopic("PositionPIDEndTrigger").publish();
    private final DoublePublisher xErrLogger = NetworkTableInstance.getDefault().getTable("logging").getDoubleTopic("X Error").publish();
    private final DoublePublisher yErrLogger = NetworkTableInstance.getDefault().getTable("logging").getDoubleTopic("Y Error").publish();
    

    public AutonPIDCommandTest(CommandSwerveDrivetrain mSwerve, Pose2d goalPose) {
        this.mSwerve = mSwerve;
        this.goalPose = goalPose;
        this.drivePID = new PIDController(2, 0, 0.01); // 1, 0, 0
        this.strafePID = new PIDController(2, 0, 0.01);

        strafeController = strafePID.calculate(mSwerve.getState().Pose.getY(), goalPose.getY());
        driveController = drivePID.calculate(mSwerve.getState().Pose.getX(), goalPose.getX());
        
        endTrigger = new Trigger(() -> {
            Pose2d diff = mSwerve.getState().Pose.relativeTo(goalPose);
            diff.getRotation().rotateBy(mSwerve.getPigeon2().getRotation2d().minus(diff.getRotation()));

            var rotation = MathUtil.isNear(
                0.0, 
                diff.getRotation().getRotations(), 
                Constants.kRotationTolerance.getRotations(), 
                0.0, 
                1.0
            );

            var position = diff.getTranslation().getNorm() < Constants.kPositionTolerance.in(Meters);

            var speed = Math.sqrt((Math.pow(mSwerve.getState().Speeds.vxMetersPerSecond, 2)) + Math.pow(mSwerve.getState().Speeds.vyMetersPerSecond, 2))  < Constants.kSpeedTolerance.in(MetersPerSecond);

            // System.out.println("end trigger conditions R: "+ rotation + "\tP: " + position + "\tS: " + speed);
            
            return rotation && position && speed;
        });

        endTriggerDebounced = endTrigger.debounce(Constants.kEndTriggerDebounce.in(Seconds));
    }

    public static Command generateCommand(CommandSwerveDrivetrain swerve, Pose2d goalPose, Time timeout){
        return new AutonPIDCommandTest(swerve, goalPose).withTimeout(timeout).finallyDo(() -> {
            swerve.setControl(new SwerveRequest.FieldCentric().withVelocityX(0).withVelocityY(0));
            swerve.applyRequest(() -> brake);
        });
    }

    @Override
    public void initialize() {
        endTriggerLogger.accept(endTrigger.getAsBoolean());
        timer.restart();
    }

    @Override
    public void execute() {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        endTriggerLogger.accept(endTrigger.getAsBoolean());

       /* if (LimelightHelpers.getTV(Constants.kLimelightName) && (Math.abs(LimelightHelpers.getTX(Constants.kLimelightName)) < 15.0)) {
            LLPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-lime");
            mSwerve.addVisionMeasurement(LLPose.pose, Utils.fpgaToCurrentTime(LLPose.timestampSeconds));
        } */

        
        
        mSwerve.applyRequest(() ->
        forwardStraight
        .withVelocityX(driveController *MaxSpeed*0.3).withVelocityY(strafeController*MaxSpeed*0.3).withTargetDirection(goalPose.getRotation()));
         /* forwardStraight
        .withVelocityX(mDriveController.calculateRobotRelativeSpeeds(LLPose.pose, goalState).vxMetersPerSecond)
        .withVelocityY(mDriveController.calculateRobotRelativeSpeeds(LLPose.pose, goalState).vyMetersPerSecond)); */      

        xErrLogger.accept(mSwerve.getState().Pose.getX() - goalPose.getX());
        yErrLogger.accept(mSwerve.getState().Pose.getY() - goalPose.getY());
    }

    @Override
    public void end(boolean interrupted) {
        endTriggerLogger.accept(endTrigger.getAsBoolean());
        timer.stop();

        Pose2d diff = mSwerve.getState().Pose.relativeTo(goalPose);

        System.out.println("Adjustments to alginment took: " + timer.get() + " seconds and interrupted = " + interrupted
            + "\nPosition offset: " + Centimeter.convertFrom(diff.getTranslation().getNorm(), Meters) + " cm"
            + "\nRotation offset: " + diff.getRotation().getMeasure().in(Degrees) + " deg"
            + "\nVelocity value: " + Math.sqrt((Math.pow(mSwerve.getState().Speeds.vxMetersPerSecond, 2)) + Math.pow(mSwerve.getState().Speeds.vyMetersPerSecond, 2)) + "m/s"
        );
    }

    @Override
    public boolean isFinished() {
        return endTriggerDebounced.getAsBoolean();
    }
}