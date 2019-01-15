package frc.team2767.twistrobot.motion;

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
import org.strykeforce.thirdcoast.trapper.Session;
import org.strykeforce.thirdcoast.trapper.SessionKt;

public class MotionController {
  private static final int DT_MS = 20;
  private static final int T1_MS = 200;
  private static final int T2_MS = 100;
  private static final double V_PROG = 5_000 * 10; // ticks/sec

  private static final double K_P = 0.01;
  private static final double OUTPUT_RANGE = 0.25;
  private static final double GOOD_ENOUGH = 5_500;
  private static final double ABS_TOL = 1.0;
  private static final DriveSubsystem drive = Robot.DRIVE;
  private final double forwardComponent;
  private final double strafeComponent;
  private final PIDController pidController;
  private final MotionProfile motionProfile;
  private final Notifier notifier;
  private final Logger logger = LoggerFactory.getLogger(this.getClass());
  private double yaw;
  private Action action;

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

    List<String> measures = List.of("profile_ticks", "actual_ticks", "actual_distance");
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

    Map<String, Object> meta =
        new HashMap<>(
            Map.of(
                "dt",
                DT_MS,
                "t1",
                T1_MS,
                "t2",
                T2_MS,
                "vProg",
                (int) V_PROG,
                "direction",
                direction,
                "azimuth",
                yaw,
                "tags",
                List.of("skippy", "rkalnins", "twist"),
                "type",
                "motion_profile",
                "k_p",
                K_P,
                "good_enough",
                GOOD_ENOUGH));

    List<List<Double>> traceData = new ArrayList<>();
    List<Double> data = new ArrayList<>();

    data.add((double) distance); // profile_ticks

    action =
        new Action(
            null, "Skippy motion profile (twist)", meta, measures, data, traceMeasures, traceData);

    Session.INSTANCE.setBaseUrl("https://keeper.strykeforce.org");
  }

  private synchronized void updateYaw(double yaw) {
    this.yaw = pidController.onTarget() ? 0 : yaw;
  }

  private void updateDrive() {
    motionProfile.calculate();
    // velocity with k_p term
    double forward, strafe, yaw;
    synchronized (this) {
      forward = forwardComponent * motionProfile.getCurrentVelocity();
      strafe = strafeComponent * motionProfile.getCurrentVelocity();
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
                motionProfile.getCurrentVelocity(), // setpoint_vel
                (double)
                    drive
                        .getAllWheels()[0]
                        .getDriveTalon()
                        .getSelectedSensorVelocity(), // actual_vel
                motionProfile.getCurrentPosition(), // profile_ticks
                (double) drive.getDistance(), // actual_ticks
                forward, // forward
                strafe, // strafe
                yaw, // yaw
                drive.getGyro().getAngle()));
  }

  public void start() {
    drive.zeroGyro();
    logger.info("START motion gyro angle = {}", drive.getGyro().getAngle());
    notifier.startPeriodic(DT_MS / 1000.0);
    pidController.enable();
    action.getMeta().put("gyroStart", drive.getGyro().getAngle());
  }

  public void stop() {
    logger.info("FINISH motion");
    drive.drive(0, 0, 0);
    notifier.stop();
    pidController.disable();

    action.getMeta().put("gyroEnd", drive.getGyro().getAngle());
    action.getData().add((double) drive.getDistance());
    SessionKt.post(action);
  }

  public boolean isFinished() {
    return motionProfile.isFinished() && pidController.onTarget();
  }
}
