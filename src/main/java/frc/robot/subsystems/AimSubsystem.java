package frc.robot.subsystems;

import frc.lib.util.MathUtil;
import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Logger;
import frc.robot.commands.DriverCommand;
import java.util.ArrayList;
import java.util.List;

public class AimSubsystem extends SubsystemBase {

    private Alliance allianceColor;
    private double[] scoringIDs = new double[3];

    public SwerveSubsystem swerve;

    // PID constants for translation and rotation — tune these!
    private final PIDController xController = new PIDController(0.1, 0, 0.01);
    private final PIDController yController = new PIDController(0.1, 0, 0.01);
    private final PIDController thController = new PIDController(0.05, 0, 0.005);
    // How close (meters) counts as "at the target"
    private static final double POSITION_TOLERANCE = 0.05;

    public AimSubsystem(SwerveSubsystem swerve) {
        this.swerve = swerve;

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

        thController.enableContinuousInput(-Math.PI, Math.PI);

    }

    // Feeds gyro heading into Limelight — required for MegaTag 2.
    private void updateLimelightOrientation() {
        LimelightHelpers.SetRobotOrientation("", swerve.getYaw().getDegrees(), 0, 0, 0, 0, 0);
    }

    // Returns the MegaTag 2 pose estimate, or null if unavailable.
    // Uses LimelightHelpers (download from Limelight's docs).

    public LimelightHelpers.PoseEstimate getMegaTag2Pose() {
        LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");

        if (estimate == null || estimate.tagCount == 0)
            return null;
        return estimate;
    }

    // Checks if it can see any valid scoring april tags
    public boolean isScoringTagVisible() {
        double tid = LimelightHelpers.getFiducialID("");
        for (double id : scoringIDs) {
            if (tid == id)
                return true;
        }
        return false;
    }

    /**
     * Drives the robot toward a target field pose using MegaTag 2.
     * Call this from a Command's execute() method.
     *
     * @param targetPose The field-relative pose to drive toward (e.g. a tag's pose)
     * @return true when the robot is within POSITION_TOLERANCE of the target
     */
    public boolean driveTowardPose(Pose2d targetPose) {
        LimelightHelpers.PoseEstimate estimate = getMegaTag2Pose();
        if (estimate == null) {
            swerve.stopModules(); // no vision data — stop safely
            return false;
        }

        Pose2d robotPose = estimate.pose;

        // Field-relative error
        double xError = targetPose.getX() - robotPose.getX();
        double yError = targetPose.getY() - robotPose.getY();
        double thError = targetPose.getRotation().getRadians() - robotPose.getRotation().getRadians();

        double xSpeed = xController.calculate(0, -xError);
        double ySpeed = yController.calculate(0, -yError);
        double thSpeed = thController.calculate(0, -thError);

        // Drive field-relative
        swerve.drive(new Translation2d(xSpeed, ySpeed), thSpeed, true);

        return Math.hypot(xError, yError) < POSITION_TOLERANCE;
    }

    /**
     * Must be called every loop. Sends the robot's current heading
     * to the Limelight so MegaTag 2 can produce a stable pose estimate.
     */
    @Override
    public void periodic() {
        updateLimelightOrientation();
    }

    public void shooting_angle() {
        double shooter_velocity = 0;
        double shooter_angle = 45;
        double y_scoring_distance = 0;
        double x_scoring_distance = 0;
        // add calling varibales from other areas ^
        double x_shooter_velocity = shooter_velocity * Math.cos(shooter_angle);
        double y_shooter_velocity = shooter_velocity * Math.sin(shooter_angle);
        double x_time_to_score = x_scoring_distance / x_shooter_velocity;
        List<Double> y_time_to_score = MathUtil.quadratic_equation(-4.905, y_shooter_velocity, y_scoring_distance);

        if (x_time_to_score > 0 && y_time_to_score.contains(x_time_to_score)) {
            Logger.Log("can score at:" + x_time_to_score);

        } else {
            Logger.Log("cannot score");
        }
    }
}