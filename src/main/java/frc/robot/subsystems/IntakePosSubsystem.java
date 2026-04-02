package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logger;
import frc.robot.Settings;
import frc.robot.swerve.Constants;

public class IntakePosSubsystem extends SubsystemBase {

    private final Timer timer = new Timer();

    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    private TalonFX motor;

    private final MotionMagicVoltage motorPosition = new MotionMagicVoltage(0);

    public IntakePosSubsystem() {
        timer.stop();
        timer.reset();

        motor = new TalonFX(Settings.INTAKE_POS_MOTOR_ID);

        // set up the motor configs
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 80;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.supplyCurrentLimitEnable;
        // armConfig.Feedback.SensorToMechanismRatio = 36.0 * 3.5; // gearbox is 3*3*4
        // and chain is 3.5

        // set PID slot 0 gains
        var slot0Configs = motorConfig.Slot0;
        slot0Configs.kP = 5.0; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output
        slot0Configs.kS = 0; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kG = 0; // gravity gain

        // set Motion Magic settings
        var motionMagicConfigs = motorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 20 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        // now set up the motor
        motor.getConfigurator().apply(motorConfig);
        motor.setPosition(-7); // assume the arm is in rest position at the start
    }

    public void setIntakePositionRotations(double position) {
        // Logger.Log("shoulder position before:" + shoulderEncoder.getPosition());
        // shoulderMotor.setReference(position.shoulderAngle);

        // Logger.Log("Shoulder motor thinks it is at " +
        // motor.getPosition().getValueAsDouble());
        motor.setControl(motorPosition.withPosition(position));
        // Logger.Log("shoulder position called with:" + position);
    }

    public void setPositionOpenTimer() {
        timer.start();
        motor.setVoltage(0.5); // clockwise
    }

    public void setPositionClosedTimer() {
        timer.start();
        motor.setVoltage(-1.8); // counterclockwise
    }

    public void silly() {
        timer.start();
        motor.setVoltage(-1);
    }

    public void setPositionOpen() {
        Logger.Log("Opening intake");
        setIntakePositionRotations(-8);
    }

    public void setPositionClosed() {
        Logger.Log("Closing intake");
        // setArmPositionRotations(0.5);
        // try {
        // Thread.sleep(2000);
        // } catch (InterruptedException e) {
        // e.printStackTrace();
        // }
        setIntakePositionRotations(0);
        motor.setPosition(-7);
    }

    public double getIntakePosition() {
        return motor.getPosition().getValueAsDouble(); // returns number in rotations
    }

    @Override
    public void periodic() {
        if (timer.get() > 0.4) {
            motor.set(0);
            timer.stop();
            timer.reset();
        }

        // Shuffleboard.getTab("LiveWindow")
        // .add("Arm Position", armMotor.getPosition().getValueAsDouble())
        // .withWidget("Simple Dial")
        // .withPosition(5, 0)
        // .withSize(1, 1);

        // SmartDashboard.putNumber("Arm Position",
        // motor.getPosition().getValueAsDouble());
    }
}
