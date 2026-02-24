package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Settings;

public class IntakeSubsystem extends SubsystemBase {

  private TalonFX cMotor;

  private TalonFXConfiguration cMotorConfig = new TalonFXConfiguration();


  public IntakeSubsystem() {

    // Assumption of use of a NEO brushless motor
    cMotor = new TalonFX(Settings.LCLIMBER_MOTOR_ID);
    cMotorConfig.CurrentLimits.SupplyCurrentLimit = 80; // was 80, 80,
    cMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.supplyCurrentLimitEnable;
    // cMotorConfig.secondaryCurrentLimit(85);


    cMotor.setPosition(0.0);

    // Update the settings
    cMotor.getConfigurator().apply(cMotorConfig);

  }

  public void run() {
    cMotor.set(Settings.CLIMBER_INTAKE_SPEED);
    // Logger.Log("lMotor current: " + lMotor.getOutputCurrent());
  }

  public void stop() {
    cMotor.set(0);
    cMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cMotor.getConfigurator().apply(cMotorConfig);
  }

}
