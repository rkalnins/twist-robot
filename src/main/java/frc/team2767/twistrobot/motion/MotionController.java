package frc.team2767.twistrobot.motion;

import static java.util.Collections.emptyList;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PIDController;
import frc.team2767.twistrobot.Robot;
import frc.team2767.twistrobot.subsystem.DriveSubsystem;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.thirdcoast.swerve.SwerveDrive;
import org.strykeforce.thirdcoast.trapper.Action;
import org.strykeforce.thirdcoast.trapper.SessionKt;

public class MotionController {
  private static final int DT_MS = 20;
  private static final int T1_MS = 200;
  private static final int T2_MS = 100;
  private static final double V_PROG = 15_000 * 10; // ticks/sec

  private static final double K_P_DRIVE = 1.2;
  private static final double K_SCALE_YAW_CORRECTION = 4.0;
  private static final double K_P_YAW = 0.01;
  private static final double K_D_YAW = 0.1;
  private static final double OUTPUT_RANGE = 0.5;
  private static final double GOOD_ENOUGH = 5_500;
  private static final double ABS_TOL = 1.0;
  private static final DriveSubsystem drive = Robot.DRIVE;
  private static double driveError;
  private static double yawError;
  private final double forwardComponent;
  private final double strafeComponent;
  private final PIDController yawController;
  private final MotionProfile motionProfile;
  private final Notifier notifier;
  private final Logger logger = LoggerFactory.getLogger(this.getClass());
  private double yaw;
  private Action action;

  private double previousAngle;
  private int distanceSetpoint;

  public MotionController(double direction, int distanceSetpoint, double yaw) {
    this.distanceSetpoint = distanceSetpoint;
    drive.setDriveMode(SwerveDrive.DriveMode.CLOSED_LOOP);
    motionProfile = new MotionProfile(DT_MS, T1_MS, T2_MS, V_PROG, distanceSetpoint);

    double ticksPerSecMax = drive.getAllWheels()[0].getDriveSetpointMax() * 10.0;
    forwardComponent = Math.cos(Math.toRadians(direction)) / ticksPerSecMax;
    strafeComponent = Math.sin(Math.toRadians(direction)) / ticksPerSecMax;
    previousAngle = drive.getGyro().getAngle();

    // yaw control
    yawController = new PIDController(K_P_YAW, 0, K_D_YAW, drive.getGyro(), this::updateYaw, 0.01);
    yawController.setSetpoint(yaw);
    yawController.setInputRange(-180d, 180d);
    yawController.setOutputRange(-OUTPUT_RANGE, OUTPUT_RANGE);
    yawController.setContinuous(true);
    yawController.setAbsoluteTolerance(ABS_TOL);
    notifier = new Notifier(this::updateDrive);

    driveError = 0.0;

    List<String> traceMeasures =
        List.of(
            "profile_acc",
            "profile_vel",
            "setpoint_vel",
            "actual_vel",
            "profile_ticks",
            "actual_ticks",
            "forward",
            "strafe",
            "yaw",
            "gyro_angle");

    logger.debug("created trace measures meta");

    Map<String, Object> meta =
        new HashMap<>(
            Map.of(
                "dt",
                DT_MS,
                "t1",
                T1_MS,
                "t2",
                T2_MS,
                "v_prog",
                (int) V_PROG,
                "direction",
                direction,
                "yaw",
                yaw,
                "tags",
                List.of("skippy", "rkalnins", "twist"),
                "type",
                "motion_profile",
                "k_p",
                K_P_DRIVE,
                "good_enough",
                GOOD_ENOUGH));

    logger.debug("created meta");

    meta.put("profile_ticks", distanceSetpoint); // Map.of takes 10 max

    List<List<Double>> traceData = new ArrayList<>();

    action =
        new Action(
            null,
            "Skippy motion profile (twist)",
            meta,
            emptyList(),
            emptyList(),
            traceMeasures,
            traceData);

    logger.debug("action created");
    logger.debug("completed keeper action logger setup");
  }

  private synchronized void updateYaw(double yaw) {
    this.yaw = yawController.onTarget() ? 0 : yaw;
  }

  private void updateDrive() {
    motionProfile.calculate();
    yawError = calculateDriveError();
    double currentDistance = drive.getDistance();
    driveError = motionProfile.getCurrentPosition() - currentDistance;

    // velocity with k_p term
    double setpointVelocity =
        motionProfile.getCurrentVelocity()
            + (driveError + yawError * K_SCALE_YAW_CORRECTION) * K_P_DRIVE;

    double forward, strafe, yaw;
    synchronized (this) {
      forward = forwardComponent * setpointVelocity;
      strafe = strafeComponent * setpointVelocity;
      yaw = this.yaw;
    }
    drive.drive(forward, strafe, yaw);

    action
        .getTraceData()
        .add(
            List.of(
                (double) (motionProfile.getIteration() * DT_MS), // millis
                motionProfile.getCurrentAcceleration(), // profile_acc
                motionProfile.getCurrentVelocity(), // profile_vel
                setpointVelocity, // setpoint_vel
                (double)
                    drive
                        .getAllWheels()[0]
                        .getDriveTalon()
                        .getSelectedSensorVelocity(), // actual_vel
                motionProfile.getCurrentPosition(), // profile_ticks
                currentDistance, // actual_ticks
                forward, // forward
                strafe, // strafe
                yaw, // yaw
                drive.getGyro().getAngle()));
  }

  private int calculateDriveError() {
    double currentAngle = drive.getGyro().getAngle();
    double angleDifference = Math.abs(previousAngle - currentAngle);
    int ticksToAdd = (int) angleDifference * drive.TICKS_PER_DEGREE;
    previousAngle = drive.getGyro().getAngle();
    return ticksToAdd;
  }

  public void start() {
    drive.zeroGyro();
    logger.info("START motion gyro angle = {}", drive.getGyro().getAngle());
    notifier.startPeriodic(DT_MS / 1000.0);
    yawController.enable();
    action.getMeta().put("gyro_start", drive.getGyro().getAngle());
  }

  public void stop() {
    logger.info("FINISH motion");
    drive.stop();
    notifier.stop();
    yawController.disable();

    action.getMeta().put("actual_ticks", drive.getDistance());
    action.getMeta().put("gyro_end", drive.getGyro().getAngle());
    SessionKt.post(action);
  }

  public boolean isFinished() {
    return motionProfile.isFinished() && yawController.onTarget();
  }
}
