package frc.team2767.twistrobot.subsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team2767.twistrobot.Robot;
import frc.team2767.twistrobot.command.TeleOpDriveCommand;
import frc.team2767.twistrobot.motion.MotionController;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.thirdcoast.swerve.SwerveDrive;
import org.strykeforce.thirdcoast.swerve.SwerveDrive.DriveMode;
import org.strykeforce.thirdcoast.swerve.SwerveDriveConfig;
import org.strykeforce.thirdcoast.swerve.Wheel;
import org.strykeforce.thirdcoast.telemetry.TelemetryService;

public class DriveSubsystem extends Subsystem {

  private static final double DRIVE_SETPOINT_MAX = 60_000.0;
  private static final int NUM_WHEELS = 4;
  private static final double ROBOT_LENGTH = 21.5;
  private static final double ROBOT_WIDTH = 21.5;
  private static final int PID = 0;
  public final int TICKS_PER_INCH = 4096;
  private final Wheel[] wheels;
  private final SwerveDrive swerve = getSwerve();
  private final Logger logger = LoggerFactory.getLogger(this.getClass());
  private int[] start = new int[4];
  private double distanceTarget;
  private MotionController motionController;

  public DriveSubsystem() {
    logger.info("drive subsystem initialized");
    wheels = swerve.getWheels();
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new TeleOpDriveCommand());
  }

  public void zeroAzimuthEncoders() {
    swerve.zeroAzimuthEncoders();
  }

  public void zeroGyro() {
    AHRS gyro = swerve.getGyro();
    gyro.setAngleAdjustment(0);
    double adj = gyro.getAngle() % 360;
    gyro.setAngleAdjustment(-adj);
    logger.info("resetting gyro zero ({})", adj);
  }

  public void resetGyroYaw() {
    swerve.getGyro().zeroYaw();
  }

  public void resetDistance() {

    logger.debug("reset wheel encoder");
    for (int i = 0; i < NUM_WHEELS; i++) {
      start[i] = wheels[i].getDriveTalon().getSelectedSensorPosition(PID);
    }
  }

  public void stop() {
    swerve.stop();
  }

  public void drive(double forward, double strafe, double azimuth) {
    swerve.drive(forward, strafe, azimuth);
  }

  public int getDistance() {
    double distance = 0;
    for (int i = 0; i < NUM_WHEELS; i++) {
      distance += Math.abs(wheels[i].getDriveTalon().getSelectedSensorPosition(PID) - start[i]);
    }
    distance /= 4;
    logger.debug("distance = {}", (int) distance);
    return (int) distance;
  }

  public AHRS getGyro() {
    return swerve.getGyro();
  }

  public void setDriveMode(DriveMode mode) {
    logger.debug("setting drive mode to {}", mode);
    swerve.setDriveMode(mode);
  }

  public void motionTo(double direction, int distance, double yaw) {
    motionController = new MotionController(direction, distance, yaw);
    motionController.start();
  }

  public boolean isMotionFinished() {
    return motionController.isFinished();
  }

  public void endMotion() {
    motionController.stop();
    motionController = null;
  }

  // Swerve configuration

  private SwerveDrive getSwerve() {
    SwerveDriveConfig config = new SwerveDriveConfig();
    config.wheels = getWheels();
    config.gyro = new AHRS(SPI.Port.kMXP);
    config.length = ROBOT_LENGTH;
    config.width = ROBOT_WIDTH;
    config.gyroLoggingEnabled = true;
    config.summarizeTalonErrors = false;

    return new SwerveDrive(config);
  }

  private Wheel[] getWheels() {
    TalonSRXConfiguration azimuthConfig = new TalonSRXConfiguration();
    azimuthConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;

    TalonSRXConfiguration driveConfig = new TalonSRXConfiguration();
    driveConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    driveConfig.continuousCurrentLimit = 40;
    driveConfig.peakCurrentDuration = 0;
    driveConfig.peakCurrentLimit = 0;
    driveConfig.slot0.kP = 0.03;
    driveConfig.slot0.kI = 0.0003;
    driveConfig.slot0.kD = 0.0;
    driveConfig.slot0.kF = 0.028;
    driveConfig.slot0.integralZone = 3_000;
    driveConfig.slot0.allowableClosedloopError = 0;
    driveConfig.motionAcceleration = 10_000;
    driveConfig.motionAcceleration = 800;
    driveConfig.velocityMeasurementPeriod = VelocityMeasPeriod.Period_25Ms;
    driveConfig.velocityMeasurementWindow = 8;

    azimuthConfig.continuousCurrentLimit = 10;
    azimuthConfig.peakCurrentDuration = 0;
    azimuthConfig.peakCurrentLimit = 0;
    azimuthConfig.slot0.kP = 10.0;
    azimuthConfig.slot0.kI = 0.0;
    azimuthConfig.slot0.kD = 100.0;
    azimuthConfig.slot0.kF = 0.0;
    azimuthConfig.slot0.integralZone = 0;
    azimuthConfig.slot0.allowableClosedloopError = 0;
    azimuthConfig.motionAcceleration = 10_000;
    azimuthConfig.motionCruiseVelocity = 800;

    TelemetryService telemetryService = Robot.TELEMETRY;
    telemetryService.stop();

    Wheel[] wheels = new Wheel[4];

    for (int i = 0; i < 4; i++) {
      TalonSRX azimuthTalon = new TalonSRX(i);
      azimuthTalon.configAllSettings(azimuthConfig);

      TalonSRX driveTalon = new TalonSRX(i + 10);
      driveTalon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 5);
      driveTalon.configAllSettings(driveConfig);
      driveTalon.setNeutralMode(NeutralMode.Brake);

      telemetryService.register(azimuthTalon);
      telemetryService.register(driveTalon);

      Wheel wheel = new Wheel(azimuthTalon, driveTalon, DRIVE_SETPOINT_MAX);
      wheels[i] = wheel;
    }

    return wheels;
  }

  public Wheel[] getAllWheels() {
    return wheels;
  }
}
