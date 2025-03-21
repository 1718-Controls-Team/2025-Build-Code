package frc.robot.commands.Auton;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;
import java.util.Set;

import frc.robot.Constants;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToReef {
    
    private final CommandSwerveDrivetrain mSwerve;

    public boolean isPIDLoopRunning = false;

    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric();


    public AlignToReef(CommandSwerveDrivetrain mSwerve) {
        this.mSwerve = mSwerve;

    }

    private PathConstraints pathConstraints = Constants.kPathContraints;

    public void changePathConstraints(PathConstraints newPathConstraints){
        this.pathConstraints = newPathConstraints;
    }

    public Command generateCommand(Pose2d target) {
        return Commands.defer(() -> {
            return getPathFromWaypoint(target);
        }, Set.of());
    }

    private Command getPathFromWaypoint(Pose2d waypoint) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(mSwerve.getState().Pose.getTranslation(), getPathVelocityHeading(mSwerve.getState().Speeds, waypoint)),
            waypoint
        );

        if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < 0.01) { //Checks if robot position is within range of target position
            return 
            Commands.sequence(
                Commands.print("start position PID loop"),
                AutonPIDCommandTest.generateCommand(mSwerve, waypoint, Constants.kAutoAlignAdjustTimeout), // Returns PID command immediately to exit command, this PID doesn't do squat most likely, it's just here for formatting
                Commands.print("end position PID loop")
            );
        }

        PathPlannerPath path = new PathPlannerPath( //Use Pathplanner to make a path
            waypoints, 
            pathConstraints,
            new IdealStartingState(getVelocityMagnitude(mSwerve.getState().Speeds), mSwerve.getRotation3d().toRotation2d()), 
            new GoalEndState(0.0, waypoint.getRotation())
        );

        path.preventFlipping = true;

        return (AutoBuilder.followPath(path).andThen( //Run path and then PID the position, probably for fine tuning
            Commands.print("start position PID loop"),
            AutonPIDCommandTest.generateCommand(mSwerve, waypoint, (
                Constants.kAutoAlignAdjustTimeout
            ))
                .beforeStarting(Commands.runOnce(() -> {isPIDLoopRunning = true;}))
                .finallyDo(() -> {isPIDLoopRunning = false;}),
            Commands.print("end position PID loop")
        )).finallyDo((interupt) -> {
            if (interupt) { //if this is false then the position pid would've X braked & called the same method
                mSwerve.setControl(forwardStraight.withVelocityX(0.0).withVelocityY(0.0));
            }
        });
    }
    

    /**
     * 
     * @param cs field relative chassis speeds
     * @return
     */
    private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target){
        if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25) {
            var diff = target.minus(mSwerve.getState().Pose).getTranslation();
            return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();//.rotateBy(Rotation2d.k180deg);
        }
        return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
    }

    private LinearVelocity getVelocityMagnitude(ChassisSpeeds cs){
        return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
    }


}