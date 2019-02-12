package frc.team2767.twistrobot.command;

import edu.wpi.first.wpilibj.command.Command;
import frc.team2767.twistrobot.Robot;
import frc.team2767.twistrobot.subsystem.DriveSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class DistanceDriveCommand extends Command {

  private static final DriveSubsystem DRIVE = Robot.DRIVE;
  private final Logger logger = LoggerFactory.getLogger(this.getClass());
  private final int distanceSetpoint;

  public DistanceDriveCommand(int distanceSetpoint) {
    this.distanceSetpoint = distanceSetpoint;
    setTimeout(12.0);
    requires(DRIVE);
  }

  @Override
  protected void initialize() {
    DRIVE.resetGyroYaw();
    DRIVE.zeroGyro();
    DRIVE.resetDistance();
    DRIVE.motionTo(0.0, distanceSetpoint, 0.0);
  }

  @Override
  protected boolean isFinished() {
    return DRIVE.isMotionFinished() || isTimedOut();
  }

  @Override
  protected void end() {
    DRIVE.endMotion();
    logger.info("MotionDrive distance = {}", DRIVE.getDistance());
  }
}
