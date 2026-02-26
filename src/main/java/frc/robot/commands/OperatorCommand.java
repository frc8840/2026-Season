package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Logger;
import frc.robot.Settings;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class OperatorCommand extends Command {

  private PS4Controller ps4controller;

  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;

  public OperatorCommand(IntakeSubsystem intake, ShooterSubsystem shooter) {
    this.intake = intake;
    this.shooter = shooter;
    // line below was missing!
    addRequirements(intake); // Default commands must require their subsystem
    addRequirements(shooter);
    ps4controller = new PS4Controller(Settings.OPERATOR_CONTROLLER_PORT);
  }

  // this gets called every X milliseconds (20ms?)
  @Override
  public void execute() {
    // intake
    if (ps4controller.getTriangleButton()) {
      Logger.Log("Triangle button pressed");
      intake.setPositionOpen();
      intake.spin();
    } else {
      intake.setPositionClosed();
      intake.stop();
    }

    // shooter
    if (ps4controller.getCircleButton()) {
      Logger.Log("Circle button pressed");
      shooter.run();
    } else {
      shooter.stop();
    }

  }
}
