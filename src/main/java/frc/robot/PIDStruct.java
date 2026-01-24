package frc.robot;

public class PIDStruct {
  public double kP;
  public double kI;
  public double kD;
  public double kF;
  public double kIZone;

  public PIDStruct(double kp, double ki, double kd) {
    this.kP = kp;
    this.kI = ki;
    this.kD = kd;
    this.kF = 0.0;
    this.kIZone = 0.0;
  }

  public PIDStruct(double kp, double ki, double kd, double kf) {
    this.kP = kp;
    this.kI = ki;
    this.kD = kd;
    this.kF = kf;
    this.kIZone = 0.0;
  }

  public PIDStruct(double kp, double ki, double kd, double kf, double kIZone) {
    this.kP = kp;
    this.kI = ki;
    this.kD = kd;
    this.kF = kf;
    this.kIZone = kIZone;
  }
}
