package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriverCommand;
import frc.robot.commands.OperatorCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakePosSubsystem;
import frc.robot.subsystems.IntakeSpinnerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private static RobotContainer instance;

  // the subsystems
  SwerveSubsystem swerveSubsystem;
  IntakeSpinnerSubsystem intakeSpinnerSubsystem;
  IntakePosSubsystem intakePosSubsystem;
  ShooterSubsystem shooterSubsystem;
  IndexerSubsystem indexerSubsystem;

  // the commands
  DriverCommand driverCommand;
  OperatorCommand operatorCommand;

  public static RobotContainer getInstance() {
    return instance;
  }

  public Command getAutoCommand() {
    return new AutoCommand(intakePosSubsystem);
  }

  public RobotContainer() {
    instance = this;

    // construct and link together the driver command
    swerveSubsystem = new SwerveSubsystem();
    driverCommand = new DriverCommand(swerveSubsystem);
    swerveSubsystem.setDefaultCommand(driverCommand);

    // construct and link together operator command
    intakeSpinnerSubsystem = new IntakeSpinnerSubsystem();
    intakePosSubsystem = new IntakePosSubsystem();
    // shooterSubsystem = new ShooterSubsystem();
    indexerSubsystem = new IndexerSubsystem();

    operatorCommand = new OperatorCommand(intakeSpinnerSubsystem, intakePosSubsystem, shooterSubsystem,
        indexerSubsystem);
    intakeSpinnerSubsystem.setDefaultCommand(operatorCommand);
    intakePosSubsystem.setDefaultCommand(operatorCommand);
    // shooterSubsystem.setDefaultCommand(operatorCommand);
    indexerSubsystem.setDefaultCommand(operatorCommand);
  }
}
