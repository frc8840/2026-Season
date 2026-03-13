package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.commands.DriverCommand;

public class AimSubsystem extends SubsystemBase {

    private Alliance allianceColor;
    private double[] scoringIDs = new double[3];

    private boolean hasTarget;
    private double tid;
    private double tx;

    public SwerveSubsystem swerve;

    public AimSubsystem() {
        allianceColor = DriverStation.getAlliance().get();
        if (allianceColor == Alliance.Red) {
            scoringIDs[0] = 8;
            scoringIDs[1] = 10;
            scoringIDs[2] = 11;
        } else if (allianceColor == Alliance.Blue) {
            scoringIDs[0] = 24;
            scoringIDs[1] = 26;
            scoringIDs[2] = 27;
        }
    }

    public void rotate(double tx) {
        // run things here make it rotate
        // swerve.drive(something, tx, true);
    }

    @Override
    public void periodic() {
        // check for valid targets
        hasTarget = LimelightHelpers.getTV("");

        if (hasTarget) {
            tid = LimelightHelpers.getFiducialID("");
            for (int i = 0; i < 3; ++i) {
                if (tid == scoringIDs[i]) {
                    tx = LimelightHelpers.getTX("");
                    rotate(tx);
                }
            }
        }
    }

}
