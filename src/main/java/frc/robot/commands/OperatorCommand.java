package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Logger;
import frc.robot.Settings;
import frc.robot.subsystems.Frankenstein3;
import frc.robot.subsystems.ShooterSubsystem;

public class OperatorCommand extends Command {

  private PS4Controller ps4controller;

  private Frankenstein3 frank;
  private ShooterSubsystem shooter;

  public OperatorCommand(Frankenstein3 frank, ShooterSubsystem shooter) {
    this.frank = frank;
    this.shooter = shooter;
    // line below was missing!
    addRequirements(frank); // Default commands must require their subsystem
    addRequirements(shooter);
    ps4controller = new PS4Controller(Settings.OPERATOR_CONTROLLER_PORT);
  }

  // this gets called every X milliseconds (20ms?)
  @Override
  public void execute() {

    // this is a test: if the triangle button is pressed, run the motor slowly, otherwise stop the motor
    if (ps4controller.getTriangleButton()) {
      Logger.Log("Triangle button pressed");
      frank.spin();
    } else {
      frank.stop();
    }

    if (ps4controller.getCircleButton()) {
      Logger.Log("Circle button pressed");
      shooter.run();
    } else {
      shooter.stop();
    }

  }
}
