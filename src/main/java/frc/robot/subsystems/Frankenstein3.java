package frc.robot.subsystems;

// Imports
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Logger;

public class Frankenstein3 extends SubsystemBase {

  private TalonFX frankMotor;
  private TalonFXConfiguration frankConfig = new TalonFXConfiguration();
 
  public void FrankSubsystem() {

    frankMotor = new TalonFX(38);

    // set up the motor configs
    frankConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    frankConfig.CurrentLimits.SupplyCurrentLimit = 70; // this is the default
    frankConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.supplyCurrentLimitEnable;

    // now set up the motor
    frankMotor.getConfigurator().apply(frankConfig);

    // now the motor is ready to be controlled
  }

  public void setSpeed(double speed) {
    frankMotor.set(speed);
  }

  public void say67() {
    Logger.Log("67!");
  }


  @Override
  public void periodic() {
  }
}