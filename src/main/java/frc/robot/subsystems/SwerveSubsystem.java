package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Logger;

public class SwerveSubsystem extends SubsystemBase {
  private final AHRS gyro;
  private SwerveDriveOdometry odometer;
  private KrakenSwerveModule[] mSwerveMods;
  private Field2d field;

  private Orchestra orchestra;

  public SwerveSubsystem() {
    gyro = new AHRS(NavXComType.kMXP_SPI);
    zeroGyro();

    SwerveModulePosition[] startPositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      startPositions[i] = new SwerveModulePosition();
    }

    odometer =
        new SwerveDriveOdometry(
            Constants.Swerve.swerveKinematics, new Rotation2d(0), startPositions);

    // Initialize swerve modules with Talon FX Krakens
    mSwerveMods =
        new KrakenSwerveModule[] {
          new KrakenSwerveModule("FL", Constants.Swerve.FLKrakenConstants),
          new KrakenSwerveModule("FR", Constants.Swerve.FRKrakenConstants),
          new KrakenSwerveModule("BL", Constants.Swerve.BLKrakenConstants),
          new KrakenSwerveModule("BR", Constants.Swerve.BRKrakenConstants)
        };

    field = new Field2d();
    orchestra = new Orchestra();
    for (KrakenSwerveModule module : mSwerveMods) {
      orchestra.addInstrument(module.angleMotor);
    }
    // var status = orchestra.loadMusic("music/Happy_Birthday.chrp");
    // Logger.Log(status.toString());
    // orchestra.play();

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) ->
            driveFromSpeeds(
                speeds), // Method that will drive the robot given ROBOT RELATIVE(speeds),
        // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds. Also optionally outputs individual module
        // feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following
            // controller for holonomic drive trains
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );
  }

  // translation and rotation are the desired behavior of the robot at this moment
  // translation vector is the desired velocity in m/s and
  // rotation is the desired angular velocity in radians per second
  public void drive(
      Translation2d translationMetersPerSecond,
      double rotationRadiansPerSecond,
      boolean fieldRelative) {
    // first, we compute our desired chassis speeds
    ChassisSpeeds chassisSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translationMetersPerSecond.getX(),
                translationMetersPerSecond.getY(),
                rotationRadiansPerSecond,
                getYaw())
            : new ChassisSpeeds(
                translationMetersPerSecond.getX(),
                translationMetersPerSecond.getY(),
                rotationRadiansPerSecond);
    driveFromSpeeds(chassisSpeeds);
  }

  // used by DriverControl and AutoBuilder
  public void driveFromSpeeds(ChassisSpeeds speeds) {
    Logger.LogPeriodic(
        "driveFromSpeeds() called with "
            + speeds.vxMetersPerSecond
            + ","
            + speeds.vyMetersPerSecond
            + " and "
            + speeds.omegaRadiansPerSecond);
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
    // do we need the below?
    // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
    setModuleStates(swerveModuleStates);
  }

  public void rotateAroundReef(boolean hugReef, double speed) {
    if (hugReef) {
      // Changing to be orthogonal to the current rotation pattern. should make it closer/farther
      // from the reef
      mSwerveMods[0].setDesiredState(
          new SwerveModuleState(speed, Rotation2d.fromRotations(0.664))); // FL
      mSwerveMods[1].setDesiredState(
          new SwerveModuleState(speed, Rotation2d.fromRotations(0.775))); // FR
      mSwerveMods[2].setDesiredState(
          new SwerveModuleState(speed, Rotation2d.fromRotations(0.836))); // BL
      mSwerveMods[3].setDesiredState(
          new SwerveModuleState(speed, Rotation2d.fromRotations(0.942))); // BR
    } else {
      mSwerveMods[0].setDesiredState(
          new SwerveModuleState(speed, Rotation2d.fromRotations(0.3))); // FL
      mSwerveMods[1].setDesiredState(
          new SwerveModuleState(speed, Rotation2d.fromRotations(0.2))); // FR
      mSwerveMods[2].setDesiredState(
          new SwerveModuleState(speed, Rotation2d.fromRotations(0.3))); // BL
      mSwerveMods[3].setDesiredState(
          new SwerveModuleState(speed, Rotation2d.fromRotations(0.2))); // BR
    }
  }

  public void printCancoderAngles() {
    for (KrakenSwerveModule module : mSwerveMods) {
      // Run the current module's motor at full speed (or desired speed).
      module.printCancoderAngle();
    }
  }

  // used by auto
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    for (int i = 0; i < mSwerveMods.length; i++) {
      mSwerveMods[i].setDesiredState(desiredStates[i]);
    }
  }

  public Pose2d getPose() {
    // Logger.LogPeriodic("Pose is " + odometer.getPoseMeters());
    return odometer.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getYaw(), getPositions(), pose);
  }

  public void zeroOdometry() {
    Rotation2d zeroedPosition = new Rotation2d(0);
    odometer.update(zeroedPosition, getPositions());
  }

  public KrakenSwerveModule[] getModules() {
    return mSwerveMods;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < mSwerveMods.length; i++) {
      positions[i] = mSwerveMods[i].getPosition();
    }
    return positions;
  }

  public SwerveModulePosition[] getZeroedPositions() {
    SwerveModulePosition[] zeroPositions = new SwerveModulePosition[4];
    for (int i = 0; i < mSwerveMods.length; i++) {
      zeroPositions[i] = new SwerveModulePosition(0.0, new Rotation2d(0.0));
    }
    return zeroPositions;
  }

  public void zeroGyro() {
    Logger.Log("zeroGyro called with " + gyro.getYaw());
     while (gyro.isCalibrating()) {
       try {
         Thread.sleep(100);
         Logger.Log("Calibrating gyro");
       } catch (InterruptedException e) {
         // nothing
       }
     }
     gyro.zeroYaw();
     Logger.Log("zeroGyro completed with " + gyro.getYaw());
  }

  public double getYawValue() {
    return gyro.getYaw(); // always returns in degrees between -180 and 180
  }

  // FROM THEIR DOCUMENTATION, NEED TO CHECK IF WE ARE HANDLING THIS THE RIGHT WAY: Returns the
  // current yaw value (in degrees, from -180 to 180)
  public Rotation2d getYaw() {
    return Constants.Swerve.invertGyro
        ? Rotation2d.fromDegrees(360 - getYawValue())
        : Rotation2d.fromDegrees(getYawValue());
  }

  @Override
  public void periodic() {
    odometer.update(getYaw(), getPositions());
    field.setRobotPose(getPose());
    for (KrakenSwerveModule mod : mSwerveMods) {
      // SmartDashboard.putNumber(
      //     "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoderAngle().getDegrees());
      // Shuffleboard.getTab("LiveWindow")
      //     .add("Mod " + mod.moduleNumber + " StateAngle", mod.getState().angle.getDegrees())
      //     .withPosition(4, 4)
      //     .withSize(1, 1);
      // SmartDashboard.putNumber(
      //     "Mod " + mod.moduleNumber + " StateAngle", mod.getState().angle.getDegrees());
      // Shuffleboard.getTab("LiveWindow")
      //     .add("Mod " + mod.moduleNumber + " Speed", mod.getState().speedMetersPerSecond)
      //     .withPosition(6, 4)
      //     .withSize(1, 1);
      // SmartDashboard.putNumber(
      //     "Mod " + mod.moduleNumber + " Speed", mod.getState().speedMetersPerSecond);
      // SmartDashboard.putNumber(
      //     "Mod " + mod.moduleNumber + " Distance", mod.getPosition().distanceMeters);
    }
    // tell dashboard where the robot thinks it is
    // Shuffleboard.getTab("LiveWindow")
    //     .add("Robot Heading", getYawValue())
    //     .withPosition(6, 2)
    //     .withSize(1, 1);
    SmartDashboard.putNumber("Robot heading:", getYawValue());
    // Shuffleboard.getTab("LiveWindow")
    //     .add("Robot Location", getPose().getTranslation().toString())
    //     .withPosition(4, 2)
    //     .withSize(1, 1);
    SmartDashboard.putString("Robot location:", getPose().getTranslation().toString());
    // SmartDashboard.putString("Module Positions: ", getPositions().toString());
    // for (KrakenSwerveModule mod : mSwerveMods) {
    //   // No voltage being sent to angleMotor, but is being sent to driveMotor
    //   Logger.Log("Module " +  mod.moduleNumber + " Angle Motor Voltage" +
    // mod.angleMotor.getMotorVoltage().getValueAsDouble());
    // }
    // try {
    //   updateSwerveEstimator();
    // } catch (NoSuchMethodError e) {
    //   System.err.println(e);
    // }

  }

  public void stopModules() {
    for (KrakenSwerveModule mod : mSwerveMods) {
      mod.stop();
    }
  }

  // constructs current estimate of chassis speeds from module encoders
  public ChassisSpeeds getChassisSpeeds() {
    SwerveModuleState frontLeftState = mSwerveMods[0].getState();
    SwerveModuleState frontRightState = mSwerveMods[1].getState();
    SwerveModuleState backLeftState = mSwerveMods[2].getState();
    SwerveModuleState backRightState = mSwerveMods[3].getState();

    // Convert to chassis speeds
    ChassisSpeeds chassisSpeeds =
        Constants.Swerve.swerveKinematics.toChassisSpeeds(
            frontLeftState, frontRightState, backLeftState, backRightState);

    return chassisSpeeds;
  }

  public Pose2d getEstimatedPose() {
    return odometer.getPoseMeters();
  }
}
