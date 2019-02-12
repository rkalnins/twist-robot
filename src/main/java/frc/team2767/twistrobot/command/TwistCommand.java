package frc.team2767.twistrobot.command;

import edu.wpi.first.wpilibj.command.Command;
import frc.team2767.twistrobot.Robot;
import frc.team2767.twistrobot.subsystem.DriveSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class TwistCommand extends Command {
  private static final DriveSubsystem DRIVE = Robot.DRIVE;
  private final Logger logger = LoggerFactory.getLogger(this.getClass());

  private int distanceSetpoint;
  private double heading;
  private double yawSetpoint;

  public TwistCommand(int distance, double heading, double endYaw) {
    this.distanceSetpoint = distance;
    this.heading = heading;
    this.yawSetpoint = endYaw;
    setTimeout(5.0);
    requires(DRIVE);
  }

  @Override
  protected void initialize() {
    DRIVE.resetGyroYaw();
    DRIVE.zeroGyro();
    DRIVE.resetDistance();
    DRIVE.motionTo(heading, distanceSetpoint, yawSetpoint);
  }

  @Override
  protected boolean isFinished() {
    return DRIVE.isMotionFinished() || isTimedOut();
  }

  @Override
  protected void end() {
    logger.debug("stopping wheels");
    DRIVE.stop();
    logger.debug("stopped wheels");
    logger.debug("ending motion");
    DRIVE.endMotion();
    logger.info("Twist command end distanceSetpoint = {}", DRIVE.getDistance());
  }
}
