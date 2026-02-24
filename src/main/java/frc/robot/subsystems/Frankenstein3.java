package frc.robot.subsystems;

// Imports
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Logger;

public class Frankenstein3 extends SubsystemBase {

  private double openPosition = 5;
  private double closedPosition = 0;
  private double spinnerSpeed = 0.1;
  private TalonFX intakePosition;
  private TalonFX spinner;
  private TalonFXConfiguration frankConfig = new TalonFXConfiguration();
 
  public Frankenstein3() {

    intakePosition = new TalonFX(23);
    spinner = new TalonFX(23);
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
    Logger.Log("spin yay");
    spinner.set(spinnerSpeed);
  }

  public void stop(){
    Logger.Log("stopping");
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