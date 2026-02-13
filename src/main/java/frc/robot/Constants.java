package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.KrakenModuleConstants;

public final class Constants {

  public static final class Climber {
    public static final double climberKP = 0.1;
    public static final double climberKI = 0.0;
    public static final double climberKD = 0.02;
    public static final double climberKFF = 0.0;
  }

  public static final class Swerve {

    public static final int pigeonID = 42;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    // our robot this year is longer than wide
    public static final double trackWidth = Units.inchesToMeters(27.5); // width
    public static final double wheelBase = Units.inchesToMeters(27.5); // length
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = 6.75; // MK4i says 6.75:1 (motor : wheel rotation)
    public static final double angleGearRatio = 21.4; // MK4i says 150/7:1 (motor : wheel rotation)

    // assuming coordinates from WPILIB from here:
    // https://www.chiefdelphi.com/t/swerve-x-and-y-flipped-in-odometry/451670/2
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), // front right
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0), // front left
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0), // back right
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0) // back left
            );

    /* Extra stuff for Kraken Motor Setup */
    public static final boolean continuousWrap = true;

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 35;
    public static final int driveContinuousCurrentLimit = 35;
    public static final boolean supplyCurrentLimitEnable = true;

    // /* Spark Angle Motor PID Values */
    // public static final double angleKP = 0.01;
    // public static final double angleKI = 0.0;
    // public static final double angleKD = 0.0;
    // public static final double angleKFF = 0.00;

    // /* Spark Drive Motor PID Values */
    // public static final double driveKP = 0.1;
    // public static final double driveKI = 0.0;
    // public static final double driveKD = 0.0;
    // public static final double driveKFF = 0.0;

    /* Kraken Drive Motor PID Values */
    public static final double dKrakenKP = 0.2;
    public static final double dKrakenKI = 0.0;
    public static final double dKrakenKD = 0.0;

    /* Kraken Angle Motor PID Values */
    public static final double aKrakenKP = 15.0;
    public static final double aKrakenKI = 0.1;
    public static final double aKrakenKD = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    // driveConversionPositionFactor is the number of rotations the motor makes per meter
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;
    // public static final double driveConversionVelocityFactor = driveConversionPositionFactor /
    // 60.0;
    // public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeedMetersPerSecond = 2.0; // meters per second
    public static final double maxAngularVelocityRadiansPerSecond =
        1.57; // radians per second, 1.57 == 90 degree turn per second

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = false;
    public static final InvertedValue angleInverted = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue driveInverted = InvertedValue.CounterClockwise_Positive;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Constants for the Kraken swerve modules */
    public static final KrakenModuleConstants FLKrakenConstants =
        new KrakenModuleConstants(34, 37, 54, Rotation2d.fromRotations(0.664));
    public static final KrakenModuleConstants FRKrakenConstants =
        new KrakenModuleConstants(29, 31, 53, Rotation2d.fromRotations(0.912));
    public static final KrakenModuleConstants BLKrakenConstants =
        new KrakenModuleConstants(25, 27, 52, Rotation2d.fromRotations(0.817));
    public static final KrakenModuleConstants BRKrakenConstants =
        new KrakenModuleConstants(35, 36, 55, Rotation2d.fromRotations(0.42));
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
  }
}
