package frc.robot;

public class Settings {

  // CLIMBER SETTINGS
  public static final int LCLIMBER_MOTOR_ID = 30;
  public static final int RCLIMBER_MOTOR_ID = 5;
  public static final double CLIMBER_OUTTAKE_SPEED = 0.5;
  public static final double CLIMBER_INTAKE_SPEED = -0.5;

  // CONTROLLER SETTINGS
  public static final int DRIVER_CONTROLLER_PORT = 0;
  public static final int OPERATOR_CONTROLLER_PORT = 1;

  // ARM SETTINGS
  public static final int ARM_MOTOR_ID = 33;
  public static final double SHOULDER_GEAR_RATIO = 266 / 1;
  public static final PIDStruct ARM_PID = new PIDStruct(10.0, 0.0, 0.1);

  // INTAKE SETTINGS
  public static final int INTAKE_POS_MOTOR_ID = 32;
  public static final int INTAKE_SPIN_MOTOR_ID = 30;
  public static final double PICKUP_OUTTAKE_SPEED = -0.2;
  public static final double PICKUP_INTAKE_SPEED = 0.25;

  // SHOOTER SETTINGS
  public static final int SHOOTER_TOP_MOTOR_ID = 21;
  public static final int SHOOTER_BOTTOM_MOTOR_ID = 20;
  public static final double SHOOTER_TOP_SPEED = 1;
  public static final double SHOOTER_BOTTOM_SPEED = -(SHOOTER_TOP_SPEED);

  // SEAN SHOOTER SETTINGS
  public static final int SEAN_SHOOTER_MOTOR_ID = 67;
  public static final double SEAN_SHOOTER_TOP_SPEED = 0.67;
}
