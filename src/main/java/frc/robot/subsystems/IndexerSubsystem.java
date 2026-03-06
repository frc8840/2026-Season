package frc.robot.subsystems;

// Imports
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Settings;

public class IndexerSubsystem extends SubsystemBase {

    private TalonFX topMotor;
    private TalonFX bottomMotor;

    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    public IndexerSubsystem() {

        // Kraken Motors - Need to Change IDs
        topMotor = new TalonFX(Settings.INDEXER_TOP_MOTOR_ID);
        bottomMotor = new TalonFX(Settings.INDEXER_BOTTOM_MOTOR_ID);

        motorConfig.CurrentLimits.SupplyCurrentLimit = 80;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.supplyCurrentLimitEnable;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Position
        topMotor.setPosition(0.0);
        bottomMotor.setPosition(0.0);

        // Updating Settings
        topMotor.getConfigurator().apply(motorConfig);
        bottomMotor.getConfigurator().apply(motorConfig);

    }

    public void indexify() {

        topMotor.set(Settings.INDEXER_TOP_SPEED);
        bottomMotor.set(Settings.INDEXER_BOTTOM_SPEED);

    }

    public void die() {

        topMotor.set(0);
        bottomMotor.set(0);
    }
}
