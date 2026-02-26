package frc.robot.subsystems;

// Imports
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Settings;

public class IntakeSubsystem extends SubsystemBase {

  private double openPosition = 5;
  private double closedPosition = 0;
  private double spinnerSpeed = 0.3;
  private TalonFX intakePosition;
  private TalonFX spinner;
  private TalonFXConfiguration motorConfig = new TalonFXConfiguration();
 
  public IntakeSubsystem() {

    intakePosition = new TalonFX(Settings.INTAKE_SLOW_MOTOR_ID);
    spinner = new TalonFX(Settings.INTAKE_FAST_MOTOR_ID);

    // set up the motor configs
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 70; // this is the default
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.supplyCurrentLimitEnable;

    // now set up the motor
    intakePosition.getConfigurator().apply(motorConfig);
    spinner.getConfigurator().apply(motorConfig);

    // now the motor is ready to be controlled
  }

  public void spin(){
    spinner.set(spinnerSpeed);
  }

  public void stop(){
    spinner.set(0);
  }

  public void setPositionOpen() {
    intakePosition.setPosition(openPosition);
  }
  public void setPositionClosed() {
    intakePosition.setPosition(closedPosition);
  }

  @Override
  public void periodic() {
  }
}