package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Logger;
import frc.robot.Settings;
import frc.robot.subsystems.SwerveSubsystem;

public class DriverCommand extends Command {

  private XboxController xboxcontroller;
  private SwerveSubsystem swerve;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(10);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(10);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(10);

  private boolean isReefRotating = false;
  private boolean isHuggingReef = false;
  private boolean fieldRelative = true;

  // Make sure the roller imported is the one from subsystems! Not from settings.
  public DriverCommand(SwerveSubsystem swerve) {
    addRequirements(swerve);

    this.swerve = swerve;
    xboxcontroller = new XboxController(Settings.DRIVER_CONTROLLER_PORT);
  }

  @Override
  public void execute() {

    if (xboxcontroller.getXButtonPressed()) {
      swerve.zeroGyro();
    }

    if (xboxcontroller.getYButtonPressed()) {
      swerve.printCancoderAngles();
    }

    if (xboxcontroller.getBButtonPressed()) {
      swerve.stopModules();
    }

    if (xboxcontroller.getAButtonPressed()) {

      isReefRotating = !isReefRotating;
      if (!isReefRotating) {
        swerve.rotateAroundReef(false, 0.0);
      }
    }
    if (xboxcontroller.getBackButtonPressed()) {
      isHuggingReef = !isHuggingReef;
    }
    if (xboxcontroller.getStartButtonPressed()) {
      fieldRelative = !fieldRelative;
    }
    // get values from the Xbox Controller joysticks
    // apply the deadband so we don't do anything right around the center of the
    // joysticks
    double translationVal =
        translationLimiter.calculate(MathUtil.applyDeadband(-xboxcontroller.getLeftY(), 0.1));
    double strafeVal =
        strafeLimiter.calculate(MathUtil.applyDeadband(xboxcontroller.getLeftX(), 0.1));
    double rotationVal =
        rotationLimiter.calculate(MathUtil.applyDeadband(xboxcontroller.getRightX(), 0.05));
    double triggerVal = xboxcontroller.getLeftTriggerAxis();

    boolean slowMode = triggerVal > 0.7;

    /* Drive */
    if (!isReefRotating) {
      Logger.LogPeriodic(
          "swerve.drive() called with translation=" + translationVal + " and strafe=" + strafeVal);
      if (slowMode) {
        swerve.drive(
            new Translation2d(translationVal, strafeVal)
                .times(Constants.Swerve.maxSpeedMetersPerSecond * 0.5), // convert to m/s
            rotationVal * Constants.Swerve.maxAngularVelocityRadiansPerSecond,
            fieldRelative);
      } else {
        swerve.drive(
            new Translation2d(translationVal, strafeVal)
                .times(Constants.Swerve.maxSpeedMetersPerSecond), // convert to m/s
            rotationVal * Constants.Swerve.maxAngularVelocityRadiansPerSecond,
            fieldRelative);
      }
      // ask for ChassisSpeeds so we can print it to logs for debugging
      ChassisSpeeds chassisSpeeds = swerve.getChassisSpeeds();
      Logger.LogPeriodic("getChassisSpeeds: " + chassisSpeeds);
    } else {
      if (isHuggingReef) {
        if (slowMode) {
          swerve.rotateAroundReef(true, translationVal * 0.5);
        } else {
          swerve.rotateAroundReef(true, translationVal);
        }
      } else {
        if (slowMode) {
          swerve.rotateAroundReef(false, strafeVal * 0.5);
        } else {
          swerve.rotateAroundReef(false, strafeVal);
        }
      }
    }
  }
}
