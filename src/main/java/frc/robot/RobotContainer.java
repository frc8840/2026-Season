package frc.robot;

import frc.robot.commands.DriverCommand;
import frc.robot.commands.OperatorCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private static RobotContainer instance;

  // the subsystems
  SwerveSubsystem swerveSubsystem;
  IntakeSubsystem intakeSubsystem;
  ShooterSubsystem shooterSubsystem;

  // the commands
  DriverCommand driverCommand;
  OperatorCommand operatorCommand;

  public static RobotContainer getInstance() {
    return instance;
  }

  public RobotContainer() {
    instance = this;

    // construct and link together operator command
    // intakeSubsystem = new IntakeSubsystem();

    shooterSubsystem = new ShooterSubsystem();
    operatorCommand = new OperatorCommand(intakeSubsystem, shooterSubsystem);
    // intakeSubsystem.setDefaultCommand(operatorCommand);
    shooterSubsystem.setDefaultCommand(operatorCommand);

    // construct and link together the driver command
    // swerveSubsystem = new SwerveSubsystem();
    // driverCommand = new DriverCommand(swerveSubsystem);
    // swerveSubsystem.setDefaultCommand(driverCommand);
  }
}
