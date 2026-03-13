package frc.robot.subsystems;

// Imports
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Settings;

public class IndexerSubsystem extends SubsystemBase {

    private TalonFX motor;
    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    public IndexerSubsystem() {

        // Kraken Motors - Need to Change IDs
        motor = new TalonFX(Settings.INDEXER_MOTOR_ID);

        motorConfig.CurrentLimits.SupplyCurrentLimit = 70;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.supplyCurrentLimitEnable;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Updating Settings
        motor.getConfigurator().apply(motorConfig);

    }

    public void indexify() {
        motor.set(Settings.INDEXER_SPEED);
    }

    public void die() {
        motor.set(0);
    }
}
