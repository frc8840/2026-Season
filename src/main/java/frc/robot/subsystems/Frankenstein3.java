package frc.robot.subsystems;

// Imports
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Frankenstein3 extends SubsystemBase {

  private double openPosition = 5;
  private double closedPosition = 0;
  private double spinnerSpeed =7;
  private TalonFX intakePosition;
  private TalonFX spinner;
  private TalonFXConfiguration frankConfig = new TalonFXConfiguration();
 
  public void FrankSubsystem() {

    intakePosition = new TalonFX(38);
    spinner = new TalonFX(38);
    // set up the motor configs
    frankConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    frankConfig.CurrentLimits.SupplyCurrentLimit = 70; // this is the default
    frankConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.supplyCurrentLimitEnable;

    // now set up the motor
    intakePosition.getConfigurator().apply(frankConfig);
    spinner.getConfigurator().apply(frankConfig);

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