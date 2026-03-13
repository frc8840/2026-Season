package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakePosSubsystem;

public class AutoCommand extends SequentialCommandGroup {
    public AutoCommand(IntakePosSubsystem intakePosSubsystem) {
        addCommands(
                new InstantCommand(() -> intakePosSubsystem.setPositionOpen(), intakePosSubsystem));
    }

}
