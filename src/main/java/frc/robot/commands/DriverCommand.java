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

  private boolean fieldRelative = true;

  // Make sure the roller imported is the one from subsystems! Not from settings.
  public DriverCommand(SwerveSubsystem swerve) {
    addRequirements(swerve); // have to do this otherwise get error "Default command must require subsystem"

    this.swerve = swerve;
    xboxcontroller = new XboxController(Settings.DRIVER_CONTROLLER_PORT);
  }

  // this gets called every X milliseconds (20ms?)
  @Override
  public void execute() {

    if (xboxcontroller.getXButtonPressed()) {
      swerve.zeroGyro();
    }

    if (xboxcontroller.getYButtonPressed()) {
      swerve.printCancoderAngles(); // for debugging
    }

    if (xboxcontroller.getBButtonPressed()) {
      swerve.stopModules();
    }

    // get desired robot velocities from the Xbox Controller joysticks
    // apply the deadband so we don't do anything right around the center of the
    // joysticks
    double translationVelocity =
        translationLimiter.calculate(MathUtil.applyDeadband(-xboxcontroller.getLeftY(), 0.1));
    double strafeVelocity =
        strafeLimiter.calculate(MathUtil.applyDeadband(xboxcontroller.getLeftX(), 0.1));
    double rotationVelocity =
        rotationLimiter.calculate(MathUtil.applyDeadband(xboxcontroller.getRightX(), 0.05));

    /* Drive */
    Logger.LogPeriodic(
          "swerve.drive() called with translation=" + translationVelocity + " and strafe=" + strafeVelocity);
      
    swerve.drive(
            new Translation2d(translationVelocity, strafeVelocity)
                .times(Constants.Swerve.maxSpeedMetersPerSecond), // convert to m/s
            rotationVelocity * Constants.Swerve.maxAngularVelocityRadiansPerSecond,
            fieldRelative);
    // ask for ChassisSpeeds so we can print it to logs for debugging
    ChassisSpeeds chassisSpeeds = swerve.getChassisSpeeds();
    Logger.LogPeriodic("getChassisSpeeds: " + chassisSpeeds);
  }
}
