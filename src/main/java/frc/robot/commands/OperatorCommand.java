package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Logger;
import frc.robot.Settings;
import frc.robot.subsystems.TestSubsystem;

public class OperatorCommand extends Command {

  private PS4Controller ps4controller;

  private TestSubsystem testSubsystem;

  public OperatorCommand(TestSubsystem testSubsystem) {
    this.testSubsystem = testSubsystem;
    // line below was missing!
    addRequirements(testSubsystem); // Default commands must require their subsystem
    ps4controller = new PS4Controller(Settings.OPERATOR_CONTROLLER_PORT);
  }

  @Override
  public void execute() {

    // this is a test: if the triangle button is pressed, run the motor slowly, otherwise stop the motor
    if (ps4controller.getTriangleButtonPressed()) {
      Logger.Log("Triangle button pressed");
      testSubsystem.setSpeed(0.1);
    } else {
      testSubsystem.setSpeed(0.0);
    }


  }
}
