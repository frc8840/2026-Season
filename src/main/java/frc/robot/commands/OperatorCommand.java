package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Logger;
import frc.robot.Settings;
import frc.robot.subsystems.IntakeSpinnerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakePosSubsystem;

public class OperatorCommand extends Command {

  private PS4Controller ps4controller;

  private IntakeSpinnerSubsystem intakeSpinner;
  private IntakePosSubsystem intakePos;
  private ShooterSubsystem shooter;
  private IndexerSubsystem indexer;

  public boolean isIntakeOpen = false;
  private int intakeIn = 0; // 0 = off, 1 = in, 2 = out
  private double shooterSpeed = 0.0;
  private int indexerIn = 0;

  public OperatorCommand(IntakeSpinnerSubsystem intakeSpinner, IntakePosSubsystem intakePos, ShooterSubsystem shooter,
      IndexerSubsystem indexer) {
    Logger.Log("operator command constructed");
    this.intakeSpinner = intakeSpinner;
    this.intakePos = intakePos;
    this.shooter = shooter;
    this.indexer = indexer;

    // line below was missing!
    // Default commands must require their subsystem
    if (intakeSpinner != null) {
      addRequirements(intakeSpinner);
    }
    if (intakePos != null) {
      addRequirements(intakePos);
    }
    if (shooter != null) {
      addRequirements(shooter);
    }
    if (indexer != null) {
      addRequirements(indexer);
    }
    ps4controller = new PS4Controller(Settings.OPERATOR_CONTROLLER_PORT);
  }

  // this gets called every X milliseconds (20ms?)
  @Override
  public void execute() {

    // intake position
    if (ps4controller.getR1ButtonPressed()) {
      Logger.Log("R1 button pressed");
      intakePos.setPositionClosedTimer();
    }
    if (ps4controller.getR2ButtonPressed()) {
      Logger.Log("R2 button pressed");
      intakePos.setPositionOpenTimer();
    }

    if (ps4controller.getCircleButtonPressed()) {
      Logger.Log("Circle button pressed");
      intakePos.silly();
    }

    // intake spinner
    if (ps4controller.getTriangleButtonPressed()) {
      Logger.Log("Triangle button pressed");
      if (intakeIn != 1) {
        intakeSpinner.spinIn();
        intakeIn = 1;
      } else {
        intakeSpinner.stop();
        intakeIn = 0;
      }
    }
    if (ps4controller.getSquareButtonPressed()) {
      Logger.Log("Square button pressed");
      if (intakeIn != 2) {
        intakeSpinner.spinOut();
        intakeIn = 2;
      } else {
        intakeSpinner.stop();
        intakeIn = 0;
      }
    }

    // indexer
    // if (ps4controller.getCrossButtonPressed()) {
    // Logger.Log("Cross button pressed");
    // if (indexerIn != 1) {
    // indexer.indexify();
    // indexerIn = 1;
    // } else {
    // indexer.die();
    // indexerIn = 0;
    // }
    // }
    // if (ps4controller.getPSButtonPressed()) {
    // Logger.Log("PS button pressed");
    // if (indexerIn != 2) {
    // indexer.indexerOut();
    // indexerIn = 2;
    // } else {
    // indexer.die();
    // indexerIn = 0;
    // }
    // }

    // shooter
    // if (ps4controller.getCircleButtonPressed()) {
    // Logger.Log("Circle button pressed");
    // if (shooterSpeed != Settings.SHOOTER_SPEED_MAX) {
    // shooter.run_max();
    // shooterSpeed = Settings.SHOOTER_SPEED_MAX;
    // } else {
    // shooter.stop();
    // shooterSpeed = 0;
    // }
    // }
    // if (ps4controller.getR1ButtonPressed()) {
    // Logger.Log("R1 button pressed");
    // if (shooterSpeed != Settings.SHOOTER_SPEED_R1) {
    // shooter.run_07();
    // shooterSpeed = Settings.SHOOTER_SPEED_R1;
    // } else {
    // shooter.stop();
    // shooterSpeed = 0;
    // }
    // }
    // if (ps4controller.getR2ButtonPressed()) {
    // Logger.Log("R2 button pressed");
    // if (shooterSpeed != Settings.SHOOTER_SPEED_R2) {
    // shooter.run_05();
    // shooterSpeed = Settings.SHOOTER_SPEED_R2;
    // } else {
    // shooter.stop();
    // shooterSpeed = 0;
    // }
    // }
  }
}