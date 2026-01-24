package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Logger;
import frc.robot.Settings;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmShooter;

public class OperatorControl extends Command {

  private PS4Controller ps4controller;

  private Arm arm;
  private ArmShooter shooter;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(10);

  private double L4ArmPosition = -37.5;
  private double L3ArmPosition = -21.0;
  private double L2ArmPosition = -13.0;
  private double L1ArmPosition = -9.0;

  public OperatorControl(Arm arm, ArmShooter shooter) {
    this.arm = arm;
    this.shooter = shooter;
    addRequirements(arm);
    addRequirements(shooter);

    ps4controller = new PS4Controller(Settings.OPERATOR_CONTROLLER_PORT);
  }

  @Override
  public void execute() {

    // drawbridge related
    // if (ps4controller.getPOV() == 0) {
    //   Logger.Log("POV top button pressed");
    //   drawbridge.open();
    // } else if (ps4controller.getPOV() == 180) {
    //   Logger.Log("POV bottom button pressed");
    //   drawbridge.close();
    // }

    // shooter/intake related
    if (ps4controller.getL2ButtonPressed()) {
      Logger.Log("L2 button pressed");
      shooter.intake();
    }
    if (ps4controller.getL1ButtonPressed()) {
      Logger.Log("L1 button pressed");
      shooter.outtake();
    }
    if (ps4controller.getR2ButtonPressed()) {
      shooter.runForward();
    }
    //   double newArmPosition = arm.getArmPosition() + 2;
    //   arm.setArmPositionRotations(newArmPosition);
    // } else if (ps4controller.getR1ButtonPressed()) {
    //   double newArmPosition = arm.getArmPosition() - 2;
    //   arm.setArmPositionRotations(newArmPosition);
    else {
      shooter.stop();
    }

    // arm position related
    if (ps4controller.getTriangleButtonPressed()) {
      Logger.Log("Triangle button pressed");
      arm.setArmPositionRotations(L4ArmPosition); // level 4
      Logger.Log("Arm position: " + arm.getArmPosition());
    }

    if (ps4controller.getSquareButtonPressed()) {
      Logger.Log("Square button pressed");
      arm.setArmPositionRotations(L3ArmPosition); // level 3
      Logger.Log("Arm position: " + arm.getArmPosition());
    }

    if (ps4controller.getCircleButtonPressed()) {
      Logger.Log("Square button pressed");
      arm.setArmPositionRotations(L2ArmPosition); // level 2
      Logger.Log("Arm position: " + arm.getArmPosition());
    }

    if (ps4controller.getCrossButtonPressed()) {
      Logger.Log("Cross button pressed");
      arm.returnToIntakePosition();
      ; // down
      Logger.Log("Arm position: " + arm.getArmPosition());
    }

    if (ps4controller.getOptionsButtonPressed()) {
      Logger.Log("Options button pressed");
      arm.setArmPositionRotations(L1ArmPosition);
    }

    if (ps4controller.getShareButtonPressed()) {
      Logger.Log("Share button pressed");
      arm.relax();
    }

    if (ps4controller.getPSButtonPressed()) {
      arm.gethard();
    }

    if (ps4controller.getR2Button()) {
      shooter.intake();
    }
    if (ps4controller.getR1Button()) {
      shooter.outtake();
    }

    // the idea here is to run the shooter fo 500ms
    // to get it up to speed, then run the intake for 1000ms
    // then top both of them
    // if (ps4controller.getTouchpadPressed()) {
    //   intake.inComplexAction = true;
    //   Command c =
    //       new SequentialCommandGroup(
    //           new InstantCommand(() -> shooter.shoot()), // run the shooter
    //           new WaitCommand(2),
    //           new InstantCommand(() -> intake.intake()), // run the intake
    //           new WaitCommand(1),
    //           new InstantCommand(
    //               () -> {
    //                 shooter.stop();
    //                 intake.stop();
    //               })); // stop them both
    //   c.schedule(); // make it happen!
    // }

    // if (ps4controller.getPSButtonPressed()) {
    //   intake.inComplexAction = true;
    //   Command c =
    //       new SequentialCommandGroup(
    //           new InstantCommand(() -> intake.intake()),
    //           new InstantCommand(() -> shooter.shoot()),
    //           new WaitCommand(0.5),
    //           new InstantCommand(
    //               () -> {
    //                 intake.stop();
    //                 shooter.stop();
    //               }));
    //   c.schedule();
    // }

    // if (ps4controller.getShareButtonPressed()) {
    //   Command c =
    //       new SequentialCommandGroup(
    //           new InstantCommand(() -> shooter.shoot()),
    //           new WaitCommand(1),
    //           new InstantCommand(
    //               () -> {
    //                 shooter.stop();
    //               }));
    // }

    // Starting to set up mode to control the arm with the left stick
    double joystickInput = MathUtil.applyDeadband(ps4controller.getLeftY(), 0.1);
    double filteredInput = translationLimiter.calculate(joystickInput);
    if (Math.abs(filteredInput) > 0.15) {
      double currentArmPosition = arm.getArmPosition();
      double newArmPosition = currentArmPosition + filteredInput * 1.0;
      arm.setArmPositionRotations(newArmPosition);
    }

    // Saves current arm position as the values to be used for the future (only this session)
    if (ps4controller.getPOV() == 0) {
      L1ArmPosition = arm.getArmPosition();
    }
    if (ps4controller.getPOV() == 90) {
      L2ArmPosition = arm.getArmPosition();
    }
    if (ps4controller.getPOV() == 180) {
      L3ArmPosition = arm.getArmPosition();
    }
    if (ps4controller.getPOV() == 270) {
      L4ArmPosition = arm.getArmPosition();
    }
  }
}
