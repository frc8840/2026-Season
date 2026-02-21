package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Logger;
import frc.robot.Settings;
import frc.robot.subsystems.Frankenstein3;

public class OperatorCommand extends Command {

  private PS4Controller ps4controller;

  private Frankenstein3 frank;

  public OperatorCommand(Frankenstein3 frank) {
    this.frank = frank;
    // line below was missing!
    addRequirements(frank); // Default commands must require their subsystem
    ps4controller = new PS4Controller(Settings.OPERATOR_CONTROLLER_PORT);
  }

  // this gets called every X milliseconds (20ms?)
  @Override
  public void execute() {

    // this is a test: if the triangle button is pressed, run the motor slowly, otherwise stop the motor
    if (ps4controller.getTriangleButtonPressed()) {
      Logger.Log("Triangle button pressed");
      frank.spin();
    } else {
      frank.stop();
    }


  }
}
