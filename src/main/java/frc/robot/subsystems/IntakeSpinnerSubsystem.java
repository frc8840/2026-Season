package frc.robot.subsystems;

// Imports
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;
import frc.robot.swerve.Constants;

public class IntakeSpinnerSubsystem extends SubsystemBase {

  private double spinnerSpeedIn = -0.4;
  private double spinnerSpeedOut = -spinnerSpeedIn;
  private TalonFX spinner;
  private TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  public IntakeSpinnerSubsystem() {

    spinner = new TalonFX(Settings.INTAKE_SPIN_MOTOR_ID);

    // set up the motor configs
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 70; // this is the default
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.supplyCurrentLimitEnable;

    // now set up the motor
    spinner.getConfigurator().apply(motorConfig);

    // now the motor is ready to be controlled
  }

  public void spinIn() {
    spinner.set(spinnerSpeedIn);
  }

  public void spinOut() {
    spinner.set(spinnerSpeedOut);
  }

  public void stop() {
    spinner.set(0);
  }

  @Override
  public void periodic() {
  }
}