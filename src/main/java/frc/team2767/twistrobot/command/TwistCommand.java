package frc.team2767.twistrobot.command;

import edu.wpi.first.wpilibj.command.Command;
import frc.team2767.twistrobot.Robot;
import frc.team2767.twistrobot.subsystem.DriveSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class TwistCommand extends Command {
  private double distanceSetpoint;
  private double heading;
  private double yawSetpoint;

  private static final DriveSubsystem drive = Robot.DRIVE;
  private final Logger logger = LoggerFactory.getLogger(this.getClass());

  public TwistCommand(double distance, double heading, double endYaw) {
    this.distanceSetpoint = distance;
    this.heading = heading;
    this.yawSetpoint = endYaw;
    requires(drive);
  }

  @Override
  protected void initialize() {
    drive.resetGyroYaw();
    drive.zeroGyro();
    drive.resetDistance();
    drive.motionTo(0.0, (int) distanceSetpoint, 0.0);
  }

  @Override
  protected boolean isFinished() {
    return drive.isMotionFinished() || isTimedOut();
  }

  @Override
  protected void end() {
    drive.endMotion();
    logger.info("MotionDrive distanceSetpoint = {}", drive.getDistance());
  }
}
