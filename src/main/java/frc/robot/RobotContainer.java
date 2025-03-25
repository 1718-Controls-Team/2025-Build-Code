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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Drive;
//import frc.robot.commands.Drive;
import frc.robot.commands.Home;
import frc.robot.commands.InitializeMechanisms;
import frc.robot.commands.Auton.AlignToReef;
import frc.robot.commands.Auton.AutonCoralPickup;
//import frc.robot.commands.Auton.AutonPIDCommandTest;
import frc.robot.commands.Auton.AutonSpitCoral;
import frc.robot.commands.Auton.VariableAutos;
import frc.robot.commands.Climber.ClimberActivate;
import frc.robot.commands.Climber.ClimberInitialize;
import frc.robot.commands.ElevatorPositions.AlgaeProcessorPos;
import frc.robot.commands.ElevatorPositions.CoralIntakePosition;
import frc.robot.commands.ElevatorPositions.L2AlgaePos;
import frc.robot.commands.ElevatorPositions.L2ScoringPosition;
import frc.robot.commands.ElevatorPositions.L3AlgaePos;
import frc.robot.commands.ElevatorPositions.L3ScoringPosition;
import frc.robot.commands.ElevatorPositions.L4ScoringPosition;
import frc.robot.commands.Intake.AlgaeDelivery;
import frc.robot.commands.Intake.AlgaePickup;
import frc.robot.commands.Intake.AlgaeSPIT;
import frc.robot.commands.Intake.CoralPickup;
import frc.robot.commands.Intake.CoralSpit;
import frc.robot.commands.ManualControls.ClimberManual;
import frc.robot.commands.ManualControls.CoralRotateManual;
import frc.robot.commands.ManualControls.ElevatorManual;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeIntake;
//import frc.robot.subsystems.BeamBreak;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.TClimber;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    //private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final TClimber m_tClimber = new TClimber();
    private final AlgaeIntake m_algaeIntake = new AlgaeIntake();
    private final Elevator m_elevator = new Elevator();
    private final CoralIntake m_coralIntake = new CoralIntake();
    private final AlignToReef m_alignmentGenerator = new AlignToReef(drivetrain);
    private final VariableAutos m_AUTOS_DONT_KILL_YOURSELVES = new VariableAutos(m_alignmentGenerator, m_coralIntake, m_elevator, m_algaeIntake);
    public Command AutonomousRun;

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);




    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    File autonFolder = new File(Filesystem.getDeployDirectory() + "/pathplanner/autos");
    private Command m_delayCommand = new WaitCommand(0.01);

    private static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public RobotContainer() {
        registerAutonCommands();
        autoChooser = AutoBuilder.buildAutoChooser("Test Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);

        autoChooser.addOption("PID Move Test", Commands.sequence(
            m_AUTOS_DONT_KILL_YOURSELVES.generateAutonCycle(new Pose2d(5.0025, 5.49527, new Rotation2d(Math.PI*4/3)), new Pose2d(5.319, 5.768, new Rotation2d(Math.PI*4/3)), new Pose2d(1.833, 6.654, new Rotation2d(2.199115)))
        ));

        autoChooser.addOption("Movement Only Test", m_alignmentGenerator.generateCommand(new Pose2d(5.0125, 5.49527, new Rotation2d(Math.PI*4/3)))); //5.12, 5.426
        autoChooser.addOption("Middle Auton", Commands.sequence(m_coralIntake.setCoralSpinCommand(10), new L4ScoringPosition(m_elevator, m_algaeIntake, m_coralIntake),
        m_alignmentGenerator.generateCommand(new Pose2d(6.010, 4.010, new Rotation2d(Math.PI*3/3))), new AutonSpitCoral(m_coralIntake)));

        configureBindings();
    }  

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            new Drive(drivetrain, driverController, m_elevator/*, m_coralIntake*/)
        );

        m_elevator.setDefaultCommand(new ElevatorManual(m_elevator, operatorController));
        m_coralIntake.setDefaultCommand(new CoralRotateManual(m_coralIntake, operatorController));
        m_tClimber.setDefaultCommand(new ClimberManual(m_tClimber, driverController));
        
        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // reset the field-centric heading on left bumper press
        driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        operatorController.y().onTrue(new L4ScoringPosition(m_elevator, m_algaeIntake, m_coralIntake));
        operatorController.x().onTrue(new L3ScoringPosition(m_elevator, m_algaeIntake, m_coralIntake));
        operatorController.b().onTrue(new L2ScoringPosition(m_elevator, m_algaeIntake, m_coralIntake));
        operatorController.a().onTrue(new Home(m_elevator, m_algaeIntake, m_coralIntake));
        operatorController.povLeft().onTrue(new L2AlgaePos(m_elevator, m_algaeIntake, m_coralIntake));
        operatorController.povRight().onTrue(new L3AlgaePos(m_elevator, m_algaeIntake, m_coralIntake));
        operatorController.rightBumper().onTrue(new AlgaeProcessorPos(m_elevator, m_algaeIntake, m_coralIntake));
        operatorController.leftBumper().onTrue(new CoralIntakePosition(m_elevator, m_algaeIntake, m_coralIntake));
        
        driverController.leftTrigger(0.5).whileTrue(new CoralPickup(m_coralIntake, m_elevator));
        driverController.rightTrigger(0.5).whileTrue(new CoralSpit(m_coralIntake));
        driverController.rightBumper().whileTrue(new AlgaeDelivery(m_algaeIntake));
        driverController.leftBumper().whileTrue(new AlgaePickup(m_algaeIntake, m_coralIntake));
        driverController.a().onTrue(new ClimberInitialize(m_elevator, m_algaeIntake, m_coralIntake, m_tClimber));
        driverController.y().whileTrue(new ClimberActivate(m_tClimber));
        driverController.x().whileTrue(new AlgaeSPIT(m_algaeIntake));

        
    }

    private void registerAutonCommands() {
        NamedCommands.registerCommand("AlgaeIntakePosition", new AlgaeProcessorPos(m_elevator, m_algaeIntake, m_coralIntake));
        NamedCommands.registerCommand("CoralIntakePosition", new CoralIntakePosition(m_elevator, m_algaeIntake, m_coralIntake));
        NamedCommands.registerCommand("L2ScoringPosition", new L2ScoringPosition(m_elevator, m_algaeIntake, m_coralIntake));
        NamedCommands.registerCommand("L3ScoringPosition", new L3ScoringPosition(m_elevator, m_algaeIntake, m_coralIntake));
        NamedCommands.registerCommand("L4ScoringPosition", new L4ScoringPosition(m_elevator, m_algaeIntake, m_coralIntake));
        NamedCommands.registerCommand("AlgaeDelivery", new AlgaeDelivery(m_algaeIntake));
        NamedCommands.registerCommand("AlgaePickup", new AlgaePickup(m_algaeIntake, m_coralIntake));

        NamedCommands.registerCommand("CoralPickup", new AutonCoralPickup(m_coralIntake));
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

    public Command runInitializeCommand() {
        return new InitializeMechanisms(m_elevator, m_algaeIntake, m_coralIntake, m_tClimber);
    }
    
    public static AprilTagFieldLayout getFieldLayout() {
        return fieldLayout;
    }
}
