package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakePosSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoCommand extends SequentialCommandGroup {
    public AutoCommand(IntakePosSubsystem intakePosSubsystem, IndexerSubsystem indexerSubsystem,
            ShooterSubsystem shooterSubsystem) {
        addCommands(
                new InstantCommand(() -> intakePosSubsystem.setPositionOpen(), intakePosSubsystem),
                new InstantCommand(() -> indexerSubsystem.indexify(), indexerSubsystem),
                new InstantCommand(() -> shooterSubsystem.run_07(), shooterSubsystem));
    }

}
