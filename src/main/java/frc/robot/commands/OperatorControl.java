package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Logger;
import frc.robot.Settings;
import frc.robot.subsystems.Test;

public class OperatorControl extends Command {

  private PS4Controller ps4controller;

  private Test test;

  public OperatorControl(Test test) {
    this.test = test;

    ps4controller = new PS4Controller(Settings.OPERATOR_CONTROLLER_PORT);
  }

  @Override
  public void execute() {


    // arm position related
    if (ps4controller.getTriangleButtonPressed()) {
      Logger.Log("Triangle button pressed");
      test.setSpeed(0.1);
    }


  }
}
