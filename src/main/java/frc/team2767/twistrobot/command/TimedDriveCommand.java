package frc.team2767.twistrobot.command;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.team2767.twistrobot.Robot;
import frc.team2767.twistrobot.subsystem.DriveSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.thirdcoast.swerve.SwerveDrive;

public class TimedDriveCommand extends TimedCommand {

  private static final DriveSubsystem DRIVE = Robot.DRIVE;

  private final Logger logger = LoggerFactory.getLogger(this.getClass());

  private final double forward;
  private final double strafe;
  private final double yaw;

  public TimedDriveCommand(double timeout, double forward, double strafe, double azimuth) {
    super(timeout);
    this.forward = forward;
    this.strafe = strafe;
    this.yaw = azimuth;
    requires(DRIVE);
  }

  @Override
  protected void initialize() {
    DRIVE.setDriveMode(SwerveDrive.DriveMode.CLOSED_LOOP);
    logger.debug("timed DRIVE init");
    DRIVE.drive(forward, strafe, yaw);
    logger.debug("forward = {}, strafe = {}, azimuth = {}", forward, strafe, yaw);
  }

  @Override
  protected void end() {
    logger.debug("ending");
    super.end();
  }
}
