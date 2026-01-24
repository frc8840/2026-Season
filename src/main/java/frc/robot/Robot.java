// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.config.CTREConfigs;

// import au.grapplerobotics.CanBridge;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static Robot instance;
  public static CTREConfigs ctreConfigs;

  RobotContainer container;
  private LaserCan lc;
  private PowerDistribution m_pdp;
  // private ShuffleboardContainer sbContainer;
  private boolean inRange = false;

  // private PhotonCamera camera;

  public static Robot getInstance() {
    return instance;
  }

  public Robot() {
    instance = this;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Logger.Log("RobotInitRan");
    ctreConfigs = new CTREConfigs();
    container = new RobotContainer();
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    lc = new LaserCan(42);
    // sbContainer = Shuffleboard.getTab("Live Window"); //change name to whatever you want
    m_pdp = new PowerDistribution(1, ModuleType.kCTRE);
    // Optionally initialise the settings of the LaserCAN, if you haven't already done so in
    // GrappleHook
    try {
      lc.setRangingMode(LaserCan.RangingMode.SHORT);
      lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
    // Creates UsbCamera and MjpegServer [1] and connects them
    HttpCamera photonCamera =
        new HttpCamera(
            "PhotonVision", "http://10.88.40.11:1181/stream.mjpg", HttpCameraKind.kMJPGStreamer);
    // PhotonCamera camera = new PhotonCamera("Arducam_OV2311_USB_CAMERA");
    // CameraServer.startAutomaticCapture(); // Change path to whatever the camera is
    // MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);
    // mjpegServer1.setSource(usbCamera);
    // Shuffleboard.getTab("Camera Stream")
    //     .addCamera("Driver Camera", "test", "mjpg:http://10.88.40.11:5800/?action=stream")
    //     .withPosition(2, 0)
    //     .withSize(3, 3);
    photonCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    CameraServer.addCamera(photonCamera);

    NetworkTableEntry streams =
        NetworkTableInstance.getDefault()
            .getTable("CameraPublisher")
            .getSubTable("PhotonVision")
            .getEntry("streams");

    streams.setStringArray(new String[] {"mjpeg:http://10.88.40.11:1181/stream.mjpg"});
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Logger.loopCounter++;
    CommandScheduler.getInstance().run();

    // CanBridge.runTCP();
    double voltage = m_pdp.getVoltage();
    // boolean added = false;
    // if (!added) {
    //   Shuffleboard.getTab("LiveWindow")
    //   .add("Voltage", voltage)
    //   .withWidget("Number Bar")
    //   .withPosition(0, 0)
    //   .withSize(1, 1);
    // }
    // sbContainer
    //     .addCamera("Camera", "Camera", "http://10.88..40.11:5800")
    //     .withWidget("Camera Stream")
    //     .withSize(4, 2)
    //     .withPosition(0, 0); // Not sure if URL can be accessed locally
    // // Get the voltage going into the PDP, in Volts.
    // // The PDP returns the voltage in increments of 0.05 Volts.
    SmartDashboard.putNumber("Voltage", voltage);
    // // Get the total current of all channels.
    // double totalCurrent = m_pdp.getTotalCurrent();
    // SmartDashboard.putNumber("Total Current", totalCurrent);
    // // Get the total power of all channels.
    // // Power is the bus voltage multiplied by the current with the units Watts.
    // double totalPower = m_pdp.getTotalPower();
    // SmartDashboard.putNumber("Total Power", totalPower);
    // // Get the total energy of all channels.
    // // Energy is the power summed over time with units Joules.
    // double totalEnergy = m_pdp.getTotalEnergy();
    // SmartDashboard.putNumber("Total Energy", totalEnergy);

    LaserCan.Measurement measurement = lc.getMeasurement();
    // if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
    // {
    //   Logger.LogPeriodic("The target is " + measurement.distance_mm + "mm away!");
    // } else {
    //   Logger.LogPeriodic(
    //       "Oh no! The target is out of range, or we can't get a reliable measurement!");
    //   // You can still use distance_mm in here, if you're ok tolerating a clamped value or an
    //   // unreliable measurement.
    // }
    if (measurement != null && measurement.distance_mm <= 400 && measurement.distance_mm >= 100) {
      inRange = true;
    } else {
      inRange = false;
    }
    // Shuffleboard.getTab("LiveWindow")
    //     .add("In Range", inRange)
    //     .withWidget("Boolean Box")
    //     .withPosition(1, 2)
    //     .withSize(1, 1);
    SmartDashboard.putBoolean("Close to Reef", inRange);

    // SmartDashboard.putData(m_pdp);
    // .add("Camera", camera)
    // .withWidget("Camera Stream")
    // .withSize(4,2);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    Command autonomousCommand = container.getAutonomousCommand();

    // Logger.Log("Autonomous Init Called");
    // Command autonomousCommand = swerve.followPathCommand("Test Path"); // Evan's alternative test
    // schedule the autonomous command - adds it to the scheduler
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // container.shooter.stop();
    // container.intake.stop();
  }

  // Line 104 to 120 can be commented out, js don't command out teleopPeriodic
  //   private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
  //    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10%
  // deadband
  //    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
  //    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  // private final XboxController m_joystick = new XboxController(0);
  // public final SwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //  m_drivetrain.setControl(
    //     m_driveRequest.withVelocityX(-joystick.getLeftY())
    //        .withVelocityY(-joystick.getLeftX())
    //        .withRotationalRate(-joystick.getRightX())
    //  );
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
