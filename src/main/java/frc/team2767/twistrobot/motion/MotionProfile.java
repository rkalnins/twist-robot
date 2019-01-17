package frc.team2767.twistrobot.motion;

import java.util.stream.DoubleStream;
import java.util.stream.IntStream;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

// Definitions, from https://www.chiefdelphi.com/forums/showpost.php?p=1204107&postcount=18
//
//    dt  = iteration time, loop period in ms
//    t1  = time for first filter, in ms
//    t2  = time for second filter, in ms
//   fl1  = filter 1 window length. fl1 = roundup(t1/dt)
//   fl2  = filter 2 window length. fl2 = roundup(t2/dt)
// v_prog = desired max speed, ticks/sec
//  dist  = travel distance, ticks
//    t4  = time to get to destination, in ms, at vProg. t4 = dist/vProg
//     n  = number of inputs to filter. n = roundup(t4/dt)

public class MotionProfile {
  private static final Logger logger = LoggerFactory.getLogger(MotionProfile.class);
  private final int dt;
  private final double v_prog;
  private final int[] f1;
  private final double[] f2;
  double curr_vel;
  double curr_pos;
  double curr_acc;
  private int n;
  private int iteration;
  private double prev_vel;
  private double prev_pos;
  private double t4;

  public MotionProfile(int dt, int t1, int t2, double v_prog, double dist) {
    this.dt = dt;
    this.v_prog = v_prog;
    f1 = new int[(int) Math.ceil((double) t1 / dt)];
    f2 = new double[(int) Math.ceil((double) t2 / dt)];
    t4 = (dist) / v_prog * 1000;
    n = (int) Math.ceil((t4 / dt));

    logger.debug("t4 = {}", t4);
    logger.debug("n = {}", n);
    logger.debug("f1.length = {}", f1.length);
    logger.debug("f2.length = {}", f2.length);
  }

  public boolean isFinished() {
    return iteration >= n + f1.length + f2.length + 1;
  }

  public void calculate() {
    int input = iteration == 0 || iteration > n ? 0 : 1;

    f1[iteration % f1.length] = input;
    double f1_out = (double) IntStream.of(f1).sum() / f1.length;
    f2[iteration % f2.length] = f1_out;
    double f2_out = DoubleStream.of(f2).sum() / f2.length;
    curr_vel = f2_out * v_prog;
    curr_pos = (((prev_vel + curr_vel) / 2) * dt) / 1000 + prev_pos;
    curr_acc = (curr_vel - prev_vel) / ((double) dt / 1000);
    prev_vel = curr_vel;
    prev_pos = curr_pos;
    iteration++;
  }

  public int getIteration() {
    return iteration;
  }

  public double getCurrentVelocity() {
    return curr_vel;
  }

  public double getCurrentPosition() {
    return curr_pos;
  }

  public double getCurrentAcceleration() {
    return curr_acc;
  }
}
