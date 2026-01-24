package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Logger;
import frc.robot.Settings;

public class Arm extends SubsystemBase {

  private TalonFXConfiguration armConfig = new TalonFXConfiguration();

  private TalonFX armMotor;

  private final MotionMagicVoltage shoulderPosition = new MotionMagicVoltage(0);

  public Arm() {

    armMotor = new TalonFX(Settings.ARM_MOTOR_ID);

    // set up the motor configs
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.CurrentLimits.SupplyCurrentLimit = 80;
    armConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.supplyCurrentLimitEnable;
    // armConfig.Feedback.SensorToMechanismRatio = 36.0 * 3.5; // gearbox is 3*3*4 and chain is 3.5

    // set PID slot 0 gains
    var slot0Configs = armConfig.Slot0;
    slot0Configs.kP = 5.0; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output
    slot0Configs.kS = 0; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kG = 0; // gravity gain

    // set Motion Magic settings
    var motionMagicConfigs = armConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 20 rps
    motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    // now set up the motor
    armMotor.getConfigurator().apply(armConfig);
    armMotor.setPosition(0); // assume the arm is in rest position at the start
  }

  public void setArmPositionRotations(double position) {
    // Logger.Log("shoulder position before:" + shoulderEncoder.getPosition());
    // shoulderMotor.setReference(position.shoulderAngle);
    Logger.Log("Shoulder motor thinks it is at " + armMotor.getPosition().getValueAsDouble());
    armMotor.setControl(shoulderPosition.withPosition(position));
    Logger.Log("shoulder position called with:" + position);
  }

  public void returnToIntakePosition() {
    Logger.Log("Returning to intake position");
    // setArmPositionRotations(0.5);
    // try {
    //   Thread.sleep(2000);
    // } catch (InterruptedException e) {
    //   e.printStackTrace();
    // }
    setArmPositionRotations(0);
  }

  public void relax() {
    Logger.Log("Relaxing arm");
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    armMotor.getConfigurator().apply(armConfig);
    armMotor.set(0);
    // elbowMotor.setIdleMode(IdleMode.kCoast);
    // elbowMotor.set(0);
  }

  public void gethard() {
    Logger.Log("Hardening arm");
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armMotor.getConfigurator().apply(armConfig);
  }

  public double getArmPosition() {
    return armMotor.getPosition().getValueAsDouble(); // returns number in rotations
  }

  @Override
  public void periodic() {
    // Shuffleboard.getTab("LiveWindow")
    //     .add("Arm Position", armMotor.getPosition().getValueAsDouble())
    //     .withWidget("Simple Dial")
    //     .withPosition(5, 0)
    //     .withSize(1, 1);
    SmartDashboard.putNumber("Arm Position", armMotor.getPosition().getValueAsDouble());
  }
}
