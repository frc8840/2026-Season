package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.config.KrakenModuleConstants;
import frc.lib.math.OnboardModuleState;
import frc.robot.Constants;
import frc.robot.Logger;
import frc.robot.Robot;

public class KrakenSwerveModule {
  public String moduleNumber;
  private Rotation2d angleOffset;

  public TalonFX angleMotor;
  public TalonFX driveMotor;
  private CANcoder angleEncoder;
  private TalonFXConfiguration angleConfig;
  private TalonFXConfiguration driveConfig;

  private final PositionVoltage anglePosition = new PositionVoltage(0).withSlot(0);

  public KrakenSwerveModule(String moduleNumber, KrakenModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.encoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new TalonFX(moduleConstants.krakenAngleID);
    angleConfig = new TalonFXConfiguration();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new TalonFX(moduleConstants.krakenDriveID);
    driveConfig = new TalonFXConfiguration();
    configDriveMotor();
  }

  public TalonFX getAngleMotor() {
    return angleMotor;
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
    setAngle(desiredState);
    setSpeed(desiredState, false);
  }

  private void resetToAbsolute() {
    double canCoderRotations = getCanCoderAngle().getRotations();
    double absolutePosition = canCoderRotations - angleOffset.getRotations();
    angleMotor.setPosition(absolutePosition);
    try {
      Thread.sleep(100);
    } catch (Exception e) {
    }
    Logger.Log(
        "angleEncoder"
            + moduleNumber
            + ": "
            + canCoderRotations
            + " - "
            + angleOffset.getRotations()
            + " = "
            + angleMotor.getPosition().getValueAsDouble());
  }

  private void configAngleMotor() {
    angleConfig.MotorOutput.Inverted = Constants.Swerve.angleInverted;
    angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // after we do this, angleMotor.getPosition will always return wheel angle in rotations
    angleConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;
    angleConfig.ClosedLoopGeneral.ContinuousWrap = Constants.Swerve.continuousWrap;

    angleConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleContinuousCurrentLimit;
    angleConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.supplyCurrentLimitEnable;

    angleConfig.Slot0.kP = Constants.Swerve.aKrakenKP;
    angleConfig.Slot0.kI = Constants.Swerve.aKrakenKI;
    angleConfig.Slot0.kD = Constants.Swerve.aKrakenKD;
    // angleConfig.Slot0.kS = 0.25;
    // angleConfig.Slot0.kV = 0.12;
    // angleConfig.Slot0.kA = 0.01;

    // need to figure out neutral mode, could be the problem
    // angleMotor.setNeutralMode(NeutralModeValue.valueOf(1));
    angleMotor.getConfigurator().apply(angleConfig);
    resetToAbsolute();
  }

  private void configAngleEncoder() {
    // CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
    angleEncoder.getConfigurator().apply(Robot.ctreConfigs.config);
  }

  private void configDriveMotor() {
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // TEG was Brake
    driveConfig.MotorOutput.Inverted = Constants.Swerve.driveInverted;

    // after we do this, driveMotor.getPosition will always return rotations of the wheel
    driveConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;
    driveConfig.ClosedLoopGeneral.ContinuousWrap = Constants.Swerve.continuousWrap;

    driveConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveContinuousCurrentLimit;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.supplyCurrentLimitEnable;

    driveConfig.Slot0.kP = Constants.Swerve.dKrakenKP;
    driveConfig.Slot0.kI = Constants.Swerve.dKrakenKI;
    driveConfig.Slot0.kD = Constants.Swerve.dKrakenKD;

    driveMotor.getConfigurator().apply(driveConfig);
    driveMotor.setPosition(0.0); // TEG: zero out the drive motor positions for odometry
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      // convert speed in m/s to percent output
      double percentOutput = desiredState.speedMetersPerSecond / 5.0;
      driveMotor.setControl(new DutyCycleOut(percentOutput));
    } else {
      // convert speed in m/s to motor rotations per second
      // VelocityVoltage doesn't seem to use the mechanism ratio
      double wheelRotationsPerSecond =
          desiredState.speedMetersPerSecond / Constants.Swerve.wheelCircumference;
      double motorRotationsPerSecond = wheelRotationsPerSecond * Constants.Swerve.driveGearRatio;
      VelocityVoltage velocityControl = new VelocityVoltage(motorRotationsPerSecond);
      velocityControl.Slot = 0;
      driveMotor.setControl(velocityControl);
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    double oldRotations = angleMotor.getPosition().getValueAsDouble();
    double newRotations = desiredState.angle.getRotations();
    if (Math.abs(newRotations - oldRotations) < 0.02) {
      return;
    }
    angleMotor.setControl(anglePosition.withPosition(newRotations));
  }

  private Rotation2d getAngle() {
    // wheel angle in rotations, because we applied the mechanism ratio already in the config
    return Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble());
  }

  public Rotation2d getCanCoderAngle() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public SwerveModuleState getState() {
    // compute wheel speed m/s from the motor velocity in rotations per second
    double speedMetersPerSecond =
        driveMotor.getVelocity().getValueAsDouble() // rotations of the wheel per second
            * Constants.Swerve.wheelCircumference; // times meters per wheel rotation
    Rotation2d angle = getAngle();
    return new SwerveModuleState(speedMetersPerSecond, angle);
  }

  public SwerveModulePosition getPosition() {
    double distanceMeters =
        driveMotor
                .getPosition()
                .getValueAsDouble() // number of wheel rotations, because we used mechanism ratio
            * Constants.Swerve.wheelCircumference; // times meters per wheel rotation
    return new SwerveModulePosition(distanceMeters, getAngle());
  }

  public void stop() {
    driveMotor.setControl(new DutyCycleOut(0));
    angleMotor.setControl(new DutyCycleOut(0));
  }

  public void printCancoderAngle() {
    Logger.Log(
        "Cancoder "
            + moduleNumber
            + ": "
            + (angleEncoder.getAbsolutePosition().getValueAsDouble() - angleOffset.getRotations()));
  }
}
