package frc.robot;

import frc.robot.commands.DriverCommand;
import frc.robot.commands.OperatorCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TestSubsystem;

public class RobotContainer {

  private static RobotContainer instance;

  // the subsystems
  SwerveSubsystem swerveSubsystem;
  TestSubsystem testSubsystem;

  // the commands
  DriverCommand driverCommand;
  OperatorCommand operatorCommand;

  public static RobotContainer getInstance() {
    return instance;
  }

  public RobotContainer() {
    instance = this;

    /*
    // construct and link together operator command
    testSubsystem = new TestSubsystem();
    operatorCommand = new OperatorCommand(testSubsystem);
    testSubsystem.setDefaultCommand(operatorCommand);
    */
    
    // construct and link together the driver command
    swerveSubsystem = new SwerveSubsystem();
    driverCommand = new DriverCommand(swerveSubsystem);
    swerveSubsystem.setDefaultCommand(driverCommand);
  }
}
