package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.DriverCommand;
import frc.robot.commands.OperatorCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmShooter;
import frc.robot.subsystems.KrakenSwerve;
import frc.robot.subsystems.TestSubsystem;
import frc.robot.subsystems.Vision;
import java.util.List;

public class RobotContainer {

  private static RobotContainer instance;

  // the subsystems
  public KrakenSwerve swerve;
  private Arm arm;
  private TestSubsystem test;
  public ArmShooter shooter;
  public TestSubsystem testSystem;
  public Vision vision;

  // the old chooser
  // private final SendableChooser<String> oldAutoChooser;
  // // for the choosing stage of pathplanner auto
  // private final SendableChooser<Command> autoChooser;

  // controllers
  DriverCommand driverControl;
  OperatorCommand operatorControl;

  // do we need this?
  TrajectoryConfig trajectoryConfig;

  // for autonomous
  // TrajectoryConfig trajectoryConfig;

  public static RobotContainer getInstance() {
    return instance;
  }

  public RobotContainer() {
    instance = this;

    // // construct the subsystems
    // swerve = new KrakenSwerve();
    // arm = new Arm();
    // shooter = new ArmShooter();

    // // set up the vision system
    // vision = new Vision(swerve);

    // Logger.Log("finished constructing subsystems, going to sleep");
    // try {
    //   Thread.sleep(2000);
    // } catch (InterruptedException e) {
    //   Logger.Log("sleep interrupted");
    // }
    // Logger.Log("finished sleeping");

    // now make the controllers
    // driverControl = new DriverControl(swerve);
    // swerve.setDefaultCommand(driverControl);
    test = new TestSubsystem();
    operatorControl = new OperatorCommand(test);
    test.setDefaultCommand(operatorControl);
    // arm.setDefaultCommand(operatorControl);
    // shooter.setDefaultCommand(operatorControl);
  }

  //   // now we set up things for auto selection and pathplanner
  //   // // these are commands that the path from pathplanner will use
  //   NamedCommands.registerCommand("Start Intake", getStartIntakeCommand());
  //   NamedCommands.registerCommand("Stop Intake", getStopIntakeCommand());
  //   NamedCommands.registerCommand("Outtake", getOuttakeCommand());
  //   NamedCommands.registerCommand("Intake Position", getIntakePositionCommand());
  //   NamedCommands.registerCommand("L2", getL2Command());
  //   NamedCommands.registerCommand("L3", getL3Command());
  //   NamedCommands.registerCommand("L4", getL4Command());
  //   NamedCommands.registerCommand("DeAlgae L2", getDeAlgaeL2Command());
  //   NamedCommands.registerCommand("Run Forward", getRunForwardCommand());

  //   // The old autonomous chooser
  //   // oldAutoChooser = new SendableChooser<>();
  //   // oldAutoChooser.setDefaultOption("Straight", "Straight");
  //   // oldAutoChooser.setDefaultOption("Left", "Left");
  //   // oldAutoChooser.setDefaultOption("Right", "Right");
  //   // oldAutoChooser.setDefaultOption("PathPlanner", "PathPlanner");
  //   // SmartDashboard.putData("Old Auto Chooser", oldAutoChooser);

  //   // The new autonomouse chooser
  //   autoChooser = AutoBuilder.buildAutoChooser();
  //   SmartDashboard.putData("Auto Chooser", autoChooser);
  //   //   newAutoChooser = AutoBuilder.buildAutoChooser();
  //   //   SmartDashboard.putData("PathPlanner Auto Chooser", newAutoChooser);

  //   trajectoryConfig =
  //       new TrajectoryConfig(
  //               Constants.AutoConstants.kMaxSpeedMetersPerSecond,
  //               Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
  //           .setKinematics(Constants.Swerve.swerveKinematics);
  // }

  public Command getAutonomousCommand() {
    return null;
  }

  // public Command getDriveForwardCommand() {
  //   Pose2d pose = new Pose2d(2, 0, new Rotation2d(0)); // straight
  //   Trajectory rollForward2Meters =
  //       TrajectoryGenerator.generateTrajectory(
  //           new Pose2d(0, 0, new Rotation2d(0)), List.of(), pose, trajectoryConfig);
  //   return new SequentialCommandGroup(
  //       new InstantCommand(() -> swerve.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
  //       getTrajectoryAutonomousCommand(rollForward2Meters),
  //       new InstantCommand(() -> swerve.stopModules()));
  // }

  // // public Command shootAndDriveForwardCommand(SimpleDirection direction) {
  // //   // get the pose for the direction
  // //   Pose2d pose = new Pose2d(0, 0, new Rotation2d(0)); // straight
  // //   if (direction == SimpleDirection.diagonalLeft) {
  // //     pose = new Pose2d(1.4, 1.4, new Rotation2d(0));
  // //   } else if (direction == SimpleDirection.diagonalRight) {
  // //     pose = new Pose2d(1.4, 1.4, new Rotation2d(0));
  // //   }

  // //   intake.inComplexAction = true;
  // //   // before we make our trajectory, let it know that we should go in reverse
  // //   Trajectory rollForward2Meters =
  // //       TrajectoryGenerator.generateTrajectory(
  // //           new Pose2d(0, 0, new Rotation2d(0)), List.of(), pose, trajectoryConfig);
  // //   return new SequentialCommandGroup(
  // //       new InstantCommand(() -> swerve.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
  // //       new InstantCommand(() -> arm.setArmPosition(ArmPosition.SPEAKERSHOOTING)),
  // //       new InstantCommand(() -> shooter.shoot()),
  // //       new WaitCommand(2),
  // //       new InstantCommand(() -> intake.intake()),
  // //       new WaitCommand(1),
  // //       new InstantCommand(
  // //           () -> {
  // //             intake.stop();
  // //             shooter.stop();
  // //           }),
  // //       new InstantCommand(() -> arm.setArmPosition(ArmPosition.REST)),
  // //       new WaitCommand(2),
  // //       getAutonomousCommand(rollForward2Meters),
  // //       new InstantCommand(() -> swerve.stopModules()));
  // // }

  // // public enum SimpleDirection {
  // //   straight,
  // //   diagonalRight,
  // //   diagonalLeft,
  // // }

  // public Command getStartIntakeCommand() {
  //   return new InstantCommand(
  //       () -> {
  //         shooter.intake();
  //       });
  // }

  // public Command getRunForwardCommand() {
  //   return new InstantCommand(
  //       () -> {
  //         shooter.runForward();
  //       });
  // }

  // public Command getStopIntakeCommand() {
  //   return new InstantCommand(
  //       () -> {
  //         shooter.stop();
  //       });
  // }

  // public Command getOuttakeCommand() {
  //   return new InstantCommand(
  //       () -> {
  //         shooter.outtake();
  //       });
  // }

  // public Command getIntakePositionCommand() {
  //   return new InstantCommand(
  //       () -> {
  //         arm.returnToIntakePosition();
  //       });
  // }

  // public Command getL2Command() {
  //   return new InstantCommand(
  //       () -> {
  //         arm.setArmPositionRotations(-13);
  //       });
  // }

  // public Command getDeAlgaeL2Command() {
  //   return new InstantCommand(
  //       () -> {
  //         arm.setArmPositionRotations(-15);
  //       });
  // }

  // public Command getL3Command() {
  //   return new InstantCommand(
  //       () -> {
  //         arm.setArmPositionRotations(-22);
  //       });
  // }

  // public Command getL4Command() {
  //   return new InstantCommand(
  //       () -> {
  //         arm.setArmPositionRotations(-36);
  //       });
  // }

  // public Command getAutonomousCommand() {
  //   // This method loads the auto when it is called, however, it is recommended
  //   // to first load your paths/autos when code starts, then return the
  //   // pre-loaded auto/path
  //   return new PathPlannerAuto("1 Middle + DeAlgae Auto");
  //   // return autoChooser.getSelected();
  // }

  // public SwerveControllerCommand getTrajectoryAutonomousCommand(Trajectory trajectory) {
  //   // create the PID controllers for feedback
  //   PIDController xController = new PIDController(0.2, 0, 0);
  //   PIDController yController = new PIDController(0.2, 0, 0);
  //   ProfiledPIDController thetaController =
  //       new ProfiledPIDController(0.2, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
  //   ;
  //   thetaController.enableContinuousInput(-Math.PI, Math.PI);
  //   return new SwerveControllerCommand(
  //       trajectory,
  //       swerve::getPose,
  //       Constants.Swerve.swerveKinematics,
  //       xController,
  //       yController,
  //       thetaController,
  //       swerve::setModuleStates,
  //       swerve);
  // }

  // public Command getPathCommand() {
  //   try {
  //     // Load the path you want to follow using its name in the GUI
  //     PathPlannerPath path = PathPlannerPath.fromPathFile("New New Path");
  //     return new SequentialCommandGroup(
  //         new InstantCommand(
  //             () -> {
  //               // swerve.zeroOdometry();
  //               swerve.resetOdometry(
  //                   new Pose2d(
  //                       7.157,
  //                       3.745,
  //                       new Rotation2d(
  //                           -3.123))); // Have to tell the reset odometry where you are in the
  //               // pathplannre start position before you start
  //               Logger.Log("PathPlannerAutoCommand() called. Current Pose" + swerve.getPose());
  //             }),

  //         // Create a path following command using AutoBuilder. This will also trigger event
  //         // markers.
  //         AutoBuilder.followPath(path));
  //   } catch (Exception e) {
  //     DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
  //     return Commands.none();
  //   }
  // }
}
