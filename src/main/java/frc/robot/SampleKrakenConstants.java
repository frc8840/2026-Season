// package frc.robot;

// import static edu.wpi.first.units.Units.Amps;
// import com.ctre.phoenix6.CANBus;
// import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.Pigeon2Configuration;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
// import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
// import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
// import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;

// public class SampleKrakenConstants {
//         // The steer motor uses any SwerveModule.SteerRequestType control request with the
//     // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
//     private static final Slot0Configs steerGains = new Slot0Configs()
//         .withKP(100).withKI(0).withKD(0.5)
//         .withKS(0.1).withKV(1.91).withKA(0)
//         .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
//     // When using closed-loop control, the drive motor uses the control
//     // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
//     private static final Slot0Configs driveGains = new Slot0Configs()
//         .withKP(0.1).withKI(0).withKD(0)
//         .withKS(0).withKV(0.124);

//     // The closed-loop output type to use for the steer motors;
//     // This affects the PID/FF gains for the steer motors
//     private static final ClosedLoopOutputType kSteerClosedLoopOutput =
// ClosedLoopOutputType.Voltage;
//     // The closed-loop output type to use for the drive motors;
//     // This affects the PID/FF gains for the drive motors
//     private static final ClosedLoopOutputType kDriveClosedLoopOutput =
// ClosedLoopOutputType.Voltage;

//     // The type of motor used for the drive motor
//     private static final DriveMotorArrangement kDriveMotorType =
// DriveMotorArrangement.TalonFX_Integrated;
//     // The type of motor used for the drive motor
//     private static final SteerMotorArrangement kSteerMotorType =
// SteerMotorArrangement.TalonFX_Integrated;

//     // The remote sensor feedback type to use for the steer motors;
//     // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to
// RemoteCANcoder
//     private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

//     // The stator current at which the wheels start to slip;
//     // This needs to be tuned to your individual robot
//     private static final double kSlipCurrent = 120.0;

//     // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be
// null.
//     // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
//     private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
//     private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
//         .withCurrentLimits(
//             new CurrentLimitsConfigs()
//                 // Swerve azimuth does not require much torque output, so we can set a relatively
// low
//                 // stator current limit to help avoid brownouts without impacting performance.
//                 .withStatorCurrentLimit(Amps.of(60))
//                 .withStatorCurrentLimitEnable(true)
//         );
//     private static final CANcoderConfiguration encoderInitialConfigs = new
// CANcoderConfiguration();
//     // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
//     private static final Pigeon2Configuration pigeonConfigs = null;

//     // CAN bus that the devices are located on;
//     // All swerve devices must share the same CAN bus
//     public static final CANBus kCANBus = new CANBus("canivore", "./logs/example.hoot");

//     // Theoretical free speed (m/s) at 12 V applied output;
//     // This needs to be tuned to your individual robot
//     public static final double kSpeedAt12Volts = 4.69;

//     // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
//     // This may need to be tuned to your individual robot
//     private static final double kCoupleRatio = 3.8181818181818183;

//     private static final double kDriveGearRatio = 7.363636363636365;
//     private static final double kSteerGearRatio = 15.42857142857143;
//     private static final double kWheelRadius = 2.167;

//     private static final boolean kInvertLeftSide = false;
//     private static final boolean kInvertRightSide = true;

//     private static final int kId = 1;

//     // These are only used for simulation
//     private static final double kSteerInertia = 0.01;
//     private static final double kDriveInertia = 0.01;
//     // Simulated voltage necessary to overcome friction
//     private static final double kSteerFrictionVoltage = (0.2);
//     private static final double kDriveFrictionVoltage = (0.2);

//     public static final SwerveDrivetrainConstants DrivetrainConstants = new
// SwerveDrivetrainConstants()
//             .withCANbusName(kCANBus.getName())
//             .withPigeon2Id(kPigeonId);
//             //Doesn't exist in the current version of the library
//             // .withPigeon2Configs(pigeonConfigs);

//     private static final SwerveModuleConstantsFactory ConstantCreator =
//         new SwerveModuleConstantsFactory()
//             .withDriveMotorGearRatio(kDriveGearRatio)
//             .withSteerMotorGearRatio(kSteerGearRatio)
//             .withCouplingGearRatio(kCoupleRatio)
//             .withWheelRadius(kWheelRadius)
//             .withSteerMotorGains(steerGains)
//             .withDriveMotorGains(driveGains)
//             .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
//             .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
//             .withSlipCurrent(kSlipCurrent)
//             .withSpeedAt12VoltsMps(kSpeedAt12Volts)
//             .withFeedbackSource(kSteerFeedbackType)
//             .withSteerInertia(kSteerInertia)
//             .withDriveInertia(kDriveInertia)
//             .withSteerFrictionVoltage(kSteerFrictionVoltage)
//             .withDriveFrictionVoltage(kDriveFrictionVoltage);

//     // Front Left
//     private static final int kFrontLeftDriveMotorId = 3;
//     private static final int kFrontLeftSteerMotorId = 2;
//     private static final int kFrontLeftEncoderId = 1;
//     private static final double kFrontLeftEncoderOffset = 0.15234375;
//     private static final boolean kFrontLeftSteerMotorInverted = true;
//     private static final boolean kFrontLeftEncoderInverted = false;

//     private static final double kFrontLeftXPos = 10;
//     private static final double kFrontLeftYPos = 10;

//     // Front Right
//     private static final int kFrontRightDriveMotorId = 1;
//     private static final int kFrontRightSteerMotorId = 0;
//     private static final int kFrontRightEncoderId = 0;
//     private static final double kFrontRightEncoderOffset = -0.4873046875;
//     private static final boolean kFrontRightSteerMotorInverted = true;
//     private static final boolean kFrontRightEncoderInverted = false;

//     private static final double kFrontRightXPos = 10;
//     private static final double kFrontRightYPos = -10;

//     // Back Left
//     private static final int kBackLeftDriveMotorId = 7;
//     private static final int kBackLeftSteerMotorId = 6;
//     private static final int kBackLeftEncoderId = 3;
//     private static final double kBackLeftEncoderOffset = -0.219482421875;
//     private static final boolean kBackLeftSteerMotorInverted = true;
//     private static final boolean kBackLeftEncoderInverted = false;

//     private static final double kBackLeftXPos = -10;
//     private static final double kBackLeftYPos = 10;

//     // Back Right
//     private static final int kBackRightDriveMotorId = 5;
//     private static final int kBackRightSteerMotorId = 4;
//     private static final int kBackRightEncoderId = 2;
//     private static final double kBackRightEncoderOffset = 0.17236328125;
//     private static final boolean kBackRightSteerMotorInverted = true;
//     private static final boolean kBackRightEncoderInverted = false;

//     private static final double kBackRightXPos = -10;
//     private static final double kBackRightYPos = -10;

//     public static final SwerveModuleConstants FrontLeft =
//         ConstantCreator.createModuleConstants(
//             kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId,
// kFrontLeftEncoderOffset,
//             kFrontLeftXPos, kFrontLeftYPos, kFrontLeftSteerMotorInverted
//         );
//     public static final SwerveModuleConstants FrontRight =
//         ConstantCreator.createModuleConstants(
//             kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId,
// kFrontRightEncoderOffset,
//             kFrontRightXPos, kFrontRightYPos, kFrontRightSteerMotorInverted
//         );
//     public static final SwerveModuleConstants BackLeft =
//         ConstantCreator.createModuleConstants(
//             kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId,
// kBackLeftEncoderOffset,
//             kBackLeftXPos, kBackLeftYPos, kBackLeftSteerMotorInverted
//         );
//     public static final SwerveModuleConstants BackRight =
//         ConstantCreator.createModuleConstants(
//             kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId,
// kBackRightEncoderOffset,
//             kBackRightXPos, kBackRightYPos, kBackRightSteerMotorInverted
//         );
// }
