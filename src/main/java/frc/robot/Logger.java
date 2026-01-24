package frc.robot;

public class Logger {

  public static void Log(String message) {
    System.out.println(loopCounter + ": " + message);
  }

  public static int loopCounter = 0; // who is incrementing this? Robot

  public static void LogPeriodic(String message) {
    if (loopCounter % 100 == 0) {
      System.out.println(loopCounter + ": " + message);
    }
  }
}
