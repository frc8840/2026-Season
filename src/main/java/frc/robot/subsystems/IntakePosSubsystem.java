package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Settings;

public class IntakePosSubsystem extends SubsystemBase {

    private final Timer timer = new Timer();

    // 20:1 gear rato? unsure if applied to spinner or pos
    private TalonFX intakePosition;
    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    public IntakePosSubsystem() {
        timer.stop();
        timer.reset();

        intakePosition = new TalonFX(Settings.INTAKE_POS_MOTOR_ID);

        // set up the motor configs
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 70; // this is the default
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.supplyCurrentLimitEnable;

        intakePosition.getConfigurator().apply(motorConfig);
    }

    public void setPositionOpen() {
        timer.start();
        intakePosition.setVoltage(-1);
    }

    public void setPositionClosed() {
        timer.start();
        intakePosition.setVoltage(1.5);
    }

    @Override
    public void periodic() {
        if (timer.get() > 1) {
            intakePosition.set(0);
            timer.stop();
            timer.reset();
        }
    }
}
