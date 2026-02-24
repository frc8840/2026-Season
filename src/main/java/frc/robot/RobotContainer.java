package frc.robot;

import frc.robot.commands.DriverCommand;
import frc.robot.commands.OperatorCommand;
import frc.robot.subsystems.Frankenstein3;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private static RobotContainer instance;

  // the subsystems
  SwerveSubsystem swerveSubsystem;
  Frankenstein3 intakeSubsystem;

  // the commands
  DriverCommand driverCommand;
  OperatorCommand operatorCommand;

  public static RobotContainer getInstance() {
    return instance;
  }

  public RobotContainer() {
    instance = this;

    // construct and link together operator command
    intakeSubsystem = new Frankenstein3();
    operatorCommand = new OperatorCommand(intakeSubsystem);
    intakeSubsystem.setDefaultCommand(operatorCommand);
    
    swerveSubsystem = new SwerveSubsystem();
    driverCommand = new DriverCommand(swerveSubsystem);
    swerveSubsystem.setDefaultCommand(driverCommand);
  }
}
