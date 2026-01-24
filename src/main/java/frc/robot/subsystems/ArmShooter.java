package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Settings;
import java.util.Date;

public class ArmShooter extends SubsystemBase {

  public TalonFX shooterMotor;
  private Date lastIntake = null;

  private final MotionMagicVoltage positionSignal = new MotionMagicVoltage(0);

  public ArmShooter() {

    shooterMotor = new TalonFX(Settings.SHOOTER_MOTOR_ID);
    var talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 40; // was 100, 80, just used 100
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.Swerve.supplyCurrentLimitEnable;

    // set PID slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = 5.0; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output
    slot0Configs.kS = 0; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kG = 0; // gravity gain

    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 20 rps
    motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    shooterMotor.getConfigurator().apply(talonFXConfigs);
  }

  @Override
  public void periodic() {
    // Shuffleboard.getTab("LiveWindow")
    //     .add("Shooter Speed", shooterMotor.getVelocity().getValueAsDouble())
    //     .withWidget("Simple Dial")
    //     .withPosition(4, 0)
    //     .withSize(1, 1);
    // SmartDashboard.putNumber("Shooter speed ", shooterMotor.getVelocity().getValueAsDouble());
  }

  // spin the motor 5 rotations forward
  public void intake() {
    double oldRotations =
        shooterMotor.getPosition().getValueAsDouble(); // current position in rotation
    shooterMotor.setControl(positionSignal.withPosition(oldRotations - 4.0));
    lastIntake = new Date();
  }

  // spin the motor 5 rotations backward
  public void outtake() {
    double oldRotations =
        shooterMotor.getPosition().getValueAsDouble(); // current position in rotation
    shooterMotor.setControl(positionSignal.withPosition(oldRotations + 6.0));
    lastIntake = new Date();
  }

  // start the motor running to a relative velocity
  public void runForward() {
    shooterMotor.set(-1.0); // 1.0 is full speed
  }

  // start the motor running to a relative velocity
  public void runBackward() {
    shooterMotor.set(0.6); // 1.0 is full speed
  }

  // stop the motor, but only if we haven't called intake() in the last second
  public void stop() {
    if (lastIntake == null) {
      shooterMotor.set(0);
    } else {
      Date now = new Date();
      long diff = now.getTime() - lastIntake.getTime();
      if (diff > 1000) { // more than a second ago
        lastIntake = null;
        shooterMotor.set(0);
      }
    }
  }
}
