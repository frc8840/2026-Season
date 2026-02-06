package frc.robot;

import frc.robot.commands.OperatorCommand;
import frc.robot.subsystems.TestSubsystem;

public class RobotContainer {

  private static RobotContainer instance;

  // the subsystems
  public TestSubsystem testSubsystem;

  // the commands
  OperatorCommand operatorCommand;

  public static RobotContainer getInstance() {
    return instance;
  }

  public RobotContainer() {
    instance = this;

    testSubsystem = new TestSubsystem();
    operatorCommand = new OperatorCommand(testSubsystem);
    testSubsystem.setDefaultCommand(operatorCommand);
  }

}
