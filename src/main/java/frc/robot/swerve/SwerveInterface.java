package frc.robot.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SwerveInterface extends Subsystem {
    void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean isOpenLoop);

    void setModuleStates(SwerveModuleState[] desiredStates);

    ChassisSpeeds getMeasuredSpeeds();

    Rotation2d getGyroYaw();

    Pose2d getPose();

    void setPose(Pose2d pose);

    default Rotation2d getHeading() {
        return getPose().getRotation();
    }

    default void setHeading(Rotation2d heading) {
        setPose(new Pose2d(getPose().getTranslation(), heading));
    }

    default void zeroHeading() {
        setHeading(new Rotation2d());
    }

    // void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds);

    /*
     * void addVisionMeasurement(Pose2d visionRobotPoseMeters, double
     * timestampSeconds,
     * Matrix<N3, N1> visionMeasurementStdDevs);
     */
}
