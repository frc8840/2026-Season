package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriverCommand;
import frc.robot.commands.OperatorCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakePosSubsystem;
import frc.robot.subsystems.IntakeSpinnerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.swerve.Constants;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.swerve.Constants.AutoConstants;
import frc.robot.swerve.Constants.Swerve;

public class RobotContainer {

  private static RobotContainer instance;

  // the subsystems
  SwerveSubsystem swerveSubsystem;
  IntakeSpinnerSubsystem intakeSpinnerSubsystem;
  IntakePosSubsystem intakePosSubsystem;
  ShooterSubsystem shooterSubsystem;
  IndexerSubsystem indexerSubsystem;

  // the commands
  DriverCommand driverCommand;
  OperatorCommand operatorCommand;

  public static RobotContainer getInstance() {
    return instance;
  }

  public Command getAutoCommand() {
    Logger.Log("getAutoCommand() called");
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(Constants.Swerve.swerveKinematics);

    List<Pose2d> waypoints = List.of(
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(Units.inchesToMeters(160), 0, Rotation2d.fromDegrees(-120)),
        new Pose2d(Units.inchesToMeters(160), Units.inchesToMeters(-90), Rotation2d.fromDegrees(-120)),
        new Pose2d(Units.inchesToMeters(105), Units.inchesToMeters(0), Rotation2d.fromDegrees(-180)),
        new Pose2d(Units.inchesToMeters(-112), Units.inchesToMeters(0), Rotation2d.fromDegrees(-180)));

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, trajectoryConfig);
    Logger.Log("trajectory generated!");
    Logger.Log("" + trajectory);

    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
        AutoConstants.kThetaControllerConstraints);
    thController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        swerveSubsystem::getPose,
        Swerve.swerveKinematics,
        xController,
        yController,
        thController,
        swerveSubsystem::setModuleStates,
        swerveSubsystem);
    Logger.Log("swerveControlledCommand defined!!!");
    Logger.Log("" + swerveControllerCommand);

    Command command = new SequentialCommandGroup(
        new InstantCommand(() -> waitFor(1)),
        new InstantCommand(() -> intakePosSubsystem.setPositionOpenTimer(), intakePosSubsystem),
        new InstantCommand(() -> intakeSpinnerSubsystem.spinIn(), intakeSpinnerSubsystem),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose()), swerveSubsystem),
        swerveControllerCommand,
        new InstantCommand(() -> swerveSubsystem.stopModules(), swerveSubsystem),
        new InstantCommand(() -> waitFor(1)));
    Logger.Log("Defined SequentialCommandGroup");
    return command;
  }

  public void waitFor(int seconds) {
    Logger.Log("waitFor starting");
    try {
      Thread.sleep(seconds * 1000);
    } catch (InterruptedException e) {
      // do nothing
    }
    Logger.Log("waitFor ending");
  }

  public RobotContainer() {
    instance = this;

    // construct and link together the driver command
    swerveSubsystem = new SwerveSubsystem();
    driverCommand = new DriverCommand(swerveSubsystem);
    swerveSubsystem.setDefaultCommand(driverCommand);

    // construct and link together operator command
    intakeSpinnerSubsystem = new IntakeSpinnerSubsystem();
    intakePosSubsystem = new IntakePosSubsystem();
    // shooterSubsystem = new ShooterSubsystem();
    // indexerSubsystem = new IndexerSubsystem();

    operatorCommand = new OperatorCommand(intakeSpinnerSubsystem, intakePosSubsystem, shooterSubsystem,
        indexerSubsystem);
    intakeSpinnerSubsystem.setDefaultCommand(operatorCommand);
    intakePosSubsystem.setDefaultCommand(operatorCommand);
    // shooterSubsystem.setDefaultCommand(operatorCommand);
    // indexerSubsystem.setDefaultCommand(operatorCommand);
  }
}
