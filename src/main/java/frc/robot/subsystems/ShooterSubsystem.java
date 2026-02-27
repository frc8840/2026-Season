package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Settings;

public class ShooterSubsystem extends SubsystemBase {

  private TalonFX topMotor;
  private TalonFX bottomMotor;

  private TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  public ShooterSubsystem() {

    // Assumption of use of a NEO brushless motor
    topMotor = new TalonFX(Settings.SHOOTER_TOP_MOTOR_ID);
    bottomMotor = new TalonFX(Settings.SHOOTER_BOTTOM_MOTOR_ID);

    motorConfig.CurrentLimits.SupplyCurrentLimit = 80; // was 80, 80,
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.supplyCurrentLimitEnable;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    topMotor.setPosition(0.0);
    bottomMotor.setPosition(0.0);

    // Update the settings
    topMotor.getConfigurator().apply(motorConfig);
    bottomMotor.getConfigurator().apply(motorConfig);

  }

  public void run() {
    topMotor.set(Settings.SHOOTER_TOP_SPEED);
    bottomMotor.set(Settings.SHOOTER_BOTTOM_SPEED);
    // Logger.Log("lMotor current: " + lMotor.getOutputCurrent());
  }

  public void stop() {
    topMotor.set(0);
    bottomMotor.set(0);
  }

}
