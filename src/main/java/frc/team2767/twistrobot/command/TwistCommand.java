package frc.team2767.twistrobot.command;

import edu.wpi.first.wpilibj.command.Command;
import frc.team2767.twistrobot.Robot;
import frc.team2767.twistrobot.subsystem.DriveSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class TwistCommand extends Command {
  private static final DriveSubsystem drive = Robot.DRIVE;
  private final Logger logger = LoggerFactory.getLogger(this.getClass());
  private int distanceSetpoint;
  private double heading;
  private double yawSetpoint;

  public TwistCommand(int distance, double heading, double endYaw) {
    this.distanceSetpoint = distance;
    this.heading = heading;
    this.yawSetpoint = endYaw;
    setTimeout(12.0);
    requires(drive);
  }

  @Override
  protected void initialize() {
    drive.resetGyroYaw();
    drive.zeroGyro();
    drive.resetDistance();
    drive.motionTo(heading, distanceSetpoint, yawSetpoint);
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