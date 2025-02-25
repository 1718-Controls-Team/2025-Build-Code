// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.File;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlgaeDelivery;
import frc.robot.commands.AlgaePickup;
import frc.robot.commands.AutonSpitCoral;
import frc.robot.commands.CancelCoralCheck;
import frc.robot.commands.ClimberActivate;
import frc.robot.commands.CoralCheckPosition;
import frc.robot.commands.CoralPickup;
import frc.robot.commands.Drive;
import frc.robot.commands.Home;
import frc.robot.commands.ElevatorPositions.AlgaeIntakePosition;
import frc.robot.commands.ElevatorPositions.CoralIntakePosition;
import frc.robot.commands.ElevatorPositions.L2ScoringPosition;
import frc.robot.commands.ElevatorPositions.L3ScoringPosition;
import frc.robot.commands.ElevatorPositions.L4ScoringPosition;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeIntake;
//import frc.robot.subsystems.BeamBreak;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.TClimber;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    @SuppressWarnings("unused")
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    @SuppressWarnings("unused")
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final TClimber m_tClimber = new TClimber();
    private final AlgaeIntake m_algaeIntake = new AlgaeIntake();
    private final Elevator m_elevator = new Elevator();
    private final CoralIntake m_coralIntake = new CoralIntake();
    public Command AutonomousRun;

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private int m_autoSelect = 0;
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    File autonFolder = new File(Filesystem.getDeployDirectory() + "/pathplanner/autos");

    private Command m_delayCommand = new WaitCommand(0.01);

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Test Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);

        registerAutonCommands();
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
            //new Drive(drivetrain, driverController)
        );
        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // reset the field-centric heading on left bumper press
        driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        operatorController.y().onTrue(new L4ScoringPosition(m_elevator, m_algaeIntake, m_coralIntake));
        operatorController.x().onTrue(new L3ScoringPosition(m_elevator, m_algaeIntake, m_coralIntake));
        operatorController.b().onTrue(new L2ScoringPosition(m_elevator, m_algaeIntake, m_coralIntake));
        operatorController.a().onTrue(new Home(m_elevator, m_algaeIntake, m_coralIntake));
        operatorController.rightBumper().onTrue(new AlgaeIntakePosition(m_elevator, m_algaeIntake, m_coralIntake));
        operatorController.leftBumper().onTrue(new CoralIntakePosition(m_elevator, m_algaeIntake, m_coralIntake));
        
        driverController.leftTrigger(0.5).whileTrue(new CoralPickup(m_coralIntake, m_elevator));
        driverController.rightTrigger(0.5).whileTrue(new CoralCheckPosition(m_coralIntake, driverController));
        driverController.rightBumper().whileTrue(new AlgaeDelivery(m_algaeIntake));
        driverController.leftBumper().whileTrue(new AlgaePickup(m_algaeIntake, m_coralIntake));
        driverController.start().onTrue( new CancelCoralCheck(m_coralIntake, m_elevator));
        driverController.y().whileTrue(new ClimberActivate(m_tClimber));
    }

    private void registerAutonCommands() {
        NamedCommands.registerCommand("AlgaeIntakePosition", new AlgaeIntakePosition(m_elevator, m_algaeIntake, m_coralIntake));
        NamedCommands.registerCommand("CoralIntakePosition", new CoralIntakePosition(m_elevator, m_algaeIntake, m_coralIntake));
        NamedCommands.registerCommand("L2ScoringPosition", new L2ScoringPosition(m_elevator, m_algaeIntake, m_coralIntake));
        NamedCommands.registerCommand("L3ScoringPosition", new L3ScoringPosition(m_elevator, m_algaeIntake, m_coralIntake));
        NamedCommands.registerCommand("L4ScoringPosition", new L4ScoringPosition(m_elevator, m_algaeIntake, m_coralIntake));
        NamedCommands.registerCommand("AlgaeDelivery", new AlgaeDelivery(m_algaeIntake));
        NamedCommands.registerCommand("AlgaeToggle", new AlgaePickup(m_algaeIntake, m_coralIntake));

        NamedCommands.registerCommand("CoralPickup", new CoralPickup(m_coralIntake, m_elevator));
        NamedCommands.registerCommand("Home", new Home(m_elevator, m_algaeIntake, m_coralIntake));
        NamedCommands.registerCommand("AutonSpitCoral", new AutonSpitCoral(m_coralIntake));
    } 
    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        AutonomousRun = m_delayCommand.andThen(autoChooser.getSelected());
        System.out.println("Auton 1 has been run");
        return AutonomousRun;
    }

    public Command getAutonomous2Command() {
        /* Run the path selected from the auto chooser */
        /*if (AutonomousRun != m_delayCommand.andThen(autoChooser.getSelected())) {
            AutonomousRun = m_delayCommand.andThen(autoChooser.getSelected());
        }*/
        return AutonomousRun;
    }
}
