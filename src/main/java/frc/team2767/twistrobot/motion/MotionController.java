package frc.team2767.twistrobot.motion;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PIDController;
import frc.team2767.twistrobot.Robot;
import frc.team2767.twistrobot.subsystem.DriveSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.thirdcoast.swerve.SwerveDrive;

public class MotionController {
  private static final int DT_MS = 20;
  private static final int T1_MS = 200;
  private static final int T2_MS = 100;
  private static final double V_PROG = 5_000 * 10; // ticks/sec

  private static final double K_P = 0.01;
  private static final double OUTPUT_RANGE = 0.25;
  private static final double ABS_TOL = 1.0;
  private static final DriveSubsystem drive = Robot.DRIVE;
  private final double forwardComponent;
  private final double strafeComponent;
  private final PIDController pidController;
  private final MotionProfile motionProfile;
  private final Notifier notifier;
  private final Logger logger = LoggerFactory.getLogger(this.getClass());
  private double yaw;

  public MotionController(double direction, int distance, double yaw) {
    drive.setDriveMode(SwerveDrive.DriveMode.CLOSED_LOOP);
    motionProfile = new MotionProfile(DT_MS, T1_MS, T2_MS, V_PROG, distance);

    double ticksPerSecMax = drive.getAllWheels()[0].getDriveSetpointMax() * 10.0;
    forwardComponent = Math.cos(Math.toRadians(direction)) / ticksPerSecMax;
    strafeComponent = Math.sin(Math.toRadians(direction)) / ticksPerSecMax;

    pidController = new PIDController(K_P, 0, 0, drive.getGyro(), this::updateYaw, 0.01);
    pidController.setSetpoint(yaw);
    pidController.setInputRange(-180d, 180d);
    pidController.setOutputRange(-OUTPUT_RANGE, OUTPUT_RANGE);
    pidController.setContinuous(true);
    pidController.setAbsoluteTolerance(ABS_TOL);

    notifier = new Notifier(this::updateDrive);
  }

  private synchronized void updateYaw(double yaw) {
    this.yaw = pidController.onTarget() ? 0 : yaw;
  }

  private void updateDrive() {
    motionProfile.calculate();
    double forward, strafe, yaw;
    synchronized (this) {
      forward = forwardComponent * motionProfile.curr_vel;
      strafe = strafeComponent * motionProfile.curr_vel;
      yaw = this.yaw;
    }
    drive.drive(forward, strafe, yaw);
  }

  public void start() {
    drive.zeroGyro();
    logger.info("START motion gyro angle = {}", drive.getGyro().getAngle());
    notifier.startPeriodic(DT_MS / 1000.0);
    pidController.enable();
  }

  public void stop() {
    logger.info("FINISH motion");
    drive.drive(0, 0, 0);
    notifier.stop();
    pidController.disable();
  }

  public boolean isFinished() {
    return motionProfile.isFinished() && pidController.onTarget();
  }
}
