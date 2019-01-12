package frc.team2767.twistrobot.motion;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import frc.team2767.twistrobot.Robot;
import frc.team2767.twistrobot.subsystem.DriveSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class YawController {

  private static final DriveSubsystem drive = Robot.DRIVE;
  private final Logger logger = LoggerFactory.getLogger(this.getClass());

  private final PIDController pid;
  private final AHRS gyro;

  private double setpoint;

  public YawController(PIDOutput output) {
    gyro = drive.getGyro();
    double p = 10.0;
    double i = 0.0;
    double d = 100.0;
    double tol = 0.0;
    double outputMax = 0.5;
    pid = new PIDController(p, i, d, gyro, output, 0.01);
    pid.setInputRange(-180d, 180d);
    pid.setOutputRange(-outputMax, outputMax);
    pid.setContinuous(true);
    pid.setAbsoluteTolerance(tol);
  }

  public boolean onTarget() {
    return pid.onTarget();
  }

  public void enable() {
    pid.enable();
  }

  public void disable() {
    pid.disable();
  }

  public double getSetpoint() {
    return setpoint;
  }

  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
    pid.setSetpoint(setpoint);
  }

  public double getAngle() {
    return gyro.getAngle();
  }
}
