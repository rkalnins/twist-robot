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
  private final double distanceSetpoint;
  private final double forward;
  private final double strafe;
  private final double yaw;

  public DistanceDriveCommand(double distanceSetpoint, double forward, double strafe, double yaw) {
    this.distanceSetpoint = distanceSetpoint;
    this.forward = forward;
    this.strafe = strafe;
    this.yaw = yaw;

    requires(drive);
  }

  @Override
  protected void initialize() {
    drive.resetDistance();
    drive.setDriveMode(SwerveDrive.DriveMode.CLOSED_LOOP);
  }

  @Override
  protected void execute() {
    drive.drive(forward, strafe, yaw);
  }

  @Override
  protected boolean isFinished() {
    return drive.getDistance() >= distanceSetpoint;
  }

  @Override
  protected void end() {
    drive.stop();
  }
}
