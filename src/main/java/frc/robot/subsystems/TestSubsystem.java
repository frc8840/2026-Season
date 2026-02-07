package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Logger;

public class TestSubsystem extends SubsystemBase {

  private TalonFX testMotor;
  private TalonFXConfiguration testConfig = new TalonFXConfiguration();

  public TestSubsystem() {

    testMotor = new TalonFX(35);

    // set up the motor configs
    testConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    testConfig.CurrentLimits.SupplyCurrentLimit = 70; // this is the default
    testConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.supplyCurrentLimitEnable;

    // now set up the motor
    testMotor.getConfigurator().apply(testConfig);

    // now the motor is ready to be controlled
  }

  public void setSpeed(double speed) {
    testMotor.set(speed);
  }

  public void sayHi() {
    Logger.Log("Hi!");
  }


  @Override
  public void periodic() {
  }
}
