package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class AimSubsystem extends SubsystemBase {

    private Optional<Alliance> allianceColor;
    private double[] scoringIDs = new double[3];

    private boolean hasTarget;
    private double tid;
    private double tx;

    public AimSubsystem() {
        allianceColor = DriverStation.getAlliance();
        if (allianceColor.get() == Alliance.Red) {
            scoringIDs[0] = 8;
            scoringIDs[1] = 10;
            scoringIDs[1] = 11;
        } else if (allianceColor.get() == Alliance.Blue) {
            scoringIDs[0] = 24;
            scoringIDs[1] = 26;
            scoringIDs[1] = 27;
        }
    }

    public void rotate(double degrees) {

    }

    @Override
    public void periodic() {
        // check for valid targets
        hasTarget = LimelightHelpers.getTV("");

        if (hasTarget) {
            tid = LimelightHelpers.getFiducialID("");
            for (int i = 0; i < 3; ++i) {
                if (tid == scoringIDs[i]) {
                    // run things here
                }
            }
        }
    }

}
