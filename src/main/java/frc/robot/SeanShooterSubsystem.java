package frc.robot;

// Imports
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Settings;

public class SeanShooterSubsystem {

    private TalonFX motor;

    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    public SeanShooterSubsystem() {

        // Assume use NEO brushless motor
        motor = new TalonFX(Settings.SEAN_SHOOTER_MOTOR_ID);

        motorConfig.CurrentLimits.SupplyCurrentLimit = 80; // was 80, 80,
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.supplyCurrentLimitEnable;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motor.setPosition(0.0);

        // Update Settings :)
        motor.getConfigurator().apply(motorConfig);

    }

    public void shoot() {

        motor.set(Settings.SEAN_SHOOTER_TOP_SPEED);

        Logger.Log("Sean's Motor is Running 6 7");

    }

    public void stop() {

        motor.set(0);

        Logger.Log("Sean's Motor is Stopping 6 7");

    }
}