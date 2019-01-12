package frc.team2767.twistrobot.command;

import edu.wpi.first.wpilibj.command.Command;
import frc.team2767.twistrobot.Robot;
import frc.team2767.twistrobot.subsystem.DriveSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.thirdcoast.swerve.SwerveDrive;

public class DistanceDriveCommand extends Command {

  private static final DriveSubsystem drive = Robot.DRIVE;
  private final Logger logger = LoggerFactory.getLogger(this.getClass());
  private final int distanceSetpoint;
  private final double forward;
  private final double strafe;
  private final double yaw;

  public DistanceDriveCommand(int distanceSetpoint, double forward, double strafe, double yaw) {
    this.distanceSetpoint = distanceSetpoint;
    this.forward = forward;
    this.strafe = strafe;
    this.yaw = yaw;

    requires(drive);
  }

  @Override
  protected void initialize() {
    drive.setDriveMode(SwerveDrive.DriveMode.CLOSED_LOOP);
    drive.resetDistance();
    drive.setDistanceTarget(distanceSetpoint);
    logger.debug("distance = {}", drive.getDistance());
    logger.debug("setpoint = {}", distanceSetpoint);

    logger.debug("f={}", forward);
    drive.drive(forward, strafe, yaw);
  }

  @Override
  protected boolean isFinished() {
    return drive.isDistanceTargetFinished();
  }

  @Override
  protected void end() {
    logger.debug("ticks traveled: {}", drive.getDistance());
    drive.stop();
  }
}
