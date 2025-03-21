package frc.robot.commands.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.ElevatorPositions.CoralIntakePosition;
import frc.robot.commands.ElevatorPositions.L4ScoringPosition;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;

public class VariableAutos {

    private AlignToReef m_alignmentGenerator;
    private CoralIntake m_CoralIntake;
    private Elevator m_Elevator;
    private AlgaeIntake m_AlgaeIntake;

    public VariableAutos(AlignToReef alignmentGenerator, CoralIntake coral, Elevator elevator, AlgaeIntake algaeIntake) {
        m_alignmentGenerator = alignmentGenerator;
        m_CoralIntake = coral;
        m_Elevator = elevator;
        m_AlgaeIntake = algaeIntake;
    }

    public Command generateAutonCycle(Pose2d scorePose, Pose2d intakePose) {
        return Commands.sequence(
            new L4ScoringPosition(m_Elevator, m_AlgaeIntake, m_CoralIntake),
            m_alignmentGenerator.generateCommand(scorePose),
            m_CoralIntake.setCoralSpinCommand(-10),
            new CoralIntakePosition(m_Elevator, m_AlgaeIntake, m_CoralIntake),
            m_alignmentGenerator.generateCommand(intakePose),
            new AutonCoralPickup(m_CoralIntake)
        );
    }

}

