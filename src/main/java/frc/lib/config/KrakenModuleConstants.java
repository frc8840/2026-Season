package frc.lib.config;

import edu.wpi.first.math.geometry.Rotation2d;

public class KrakenModuleConstants {
  public final int krakenDriveID;
  public final int krakenAngleID;
  public final int encoderID;
  public final Rotation2d angleOffset;

  /**
   * Swerve Module Constants to be used when creating swerve modules.
   *
   * @param krakenDriveID
   * @param krakenAngleID
   * @param encoderID
   * @param angleOffset
   */
  public KrakenModuleConstants(
      int krakenDriveID, int krakenAngleID, int encoderID, Rotation2d angleOffset) {

    this.krakenDriveID = krakenDriveID;
    this.krakenAngleID = krakenAngleID;
    this.encoderID = encoderID;
    this.angleOffset = angleOffset;
  }
}
