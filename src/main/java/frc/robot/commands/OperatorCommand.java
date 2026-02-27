package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Logger;
import frc.robot.SeanShooterSubsystem;
import frc.robot.Settings;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class OperatorCommand extends Command {

  private PS4Controller ps4controller;

  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;
  private SeanShooterSubsystem seanShooter;

  private boolean isIntakeOpen = false;
  private boolean isIntakeSpinnerOn = false;
  private boolean isShooterOn = false;

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
    // intake pos
    if (ps4controller.getTriangleButtonPressed()) {
      Logger.Log("Triangle button pressed");
      if (!isIntakeOpen) {
        intake.setPositionOpen();
      } else {
        intake.setPositionClosed();
      }
      isIntakeOpen = !isIntakeOpen;
    }

    // intake spinner
    if (ps4controller.getSquareButtonPressed()) {
      Logger.Log("Square button pressed");
      if (!isIntakeSpinnerOn) {
        intake.spin();
      } else {
        intake.stop();
      }
      isIntakeSpinnerOn = !isIntakeSpinnerOn;
    }

    // shooter
    if (ps4controller.getCircleButtonPressed()) {
      Logger.Log("Circle button pressed");
      if (!isShooterOn) {
        shooter.run();
      } else {
        shooter.stop();
      }
      isShooterOn = !isShooterOn;
    }

    // Sean's Shooter
    if (ps4controller.getCircleButtonPressed()) {
      if (!isShooterOn) {
        seanShooter.shoot();
      } else {
        seanShooter.stop();
      }
      isShooterOn = !isShooterOn;
    }

  }
}
