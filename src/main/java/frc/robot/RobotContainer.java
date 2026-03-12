package frc.robot;

import frc.robot.commands.DriverCommand;
import frc.robot.commands.OperatorCommand;
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

  // the commands
  DriverCommand driverCommand;
  OperatorCommand operatorCommand;

  public static RobotContainer getInstance() {
    return instance;
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

    operatorCommand = new OperatorCommand(intakeSpinnerSubsystem, intakePosSubsystem, shooterSubsystem);
    intakeSpinnerSubsystem.setDefaultCommand(operatorCommand);
    intakePosSubsystem.setDefaultCommand(operatorCommand);
    // shooterSubsystem.setDefaultCommand(operatorCommand);
  }
}
