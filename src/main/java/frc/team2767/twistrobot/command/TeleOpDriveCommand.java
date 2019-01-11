package frc.team2767.twistrobot.command;

import static org.strykeforce.thirdcoast.swerve.SwerveDrive.DriveMode.TELEOP;

import edu.wpi.first.wpilibj.command.Command;
import frc.team2767.twistrobot.Robot;
import frc.team2767.twistrobot.control.DriverControls;
import frc.team2767.twistrobot.subsystem.DriveSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public final class TeleOpDriveCommand extends Command {

  private static final double DEADBAND = 0.05;
  private static final DriveSubsystem drive = Robot.DRIVE;
  private static final DriverControls controls = Robot.CONTROLS.getDriverControls();
  private final Logger logger = LoggerFactory.getLogger(this.getClass());

  public TeleOpDriveCommand() {
    requires(drive);
  }

  @Override
  protected void initialize() {
    drive.setDriveMode(TELEOP);
  }

  @Override
  protected void execute() {

    double forward = deadband(controls.getForward());
    double strafe = deadband(controls.getStrafe());
    double yaw = deadband(controls.getYaw());
    //    double yaw = 0.0; // not used

    drive.drive(forward, strafe, yaw);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    drive.drive(0.0, 0.0, 0.0);
  }

  private double deadband(double value) {
    if (Math.abs(value) < DEADBAND) return 0.0;
    return value;
  }
}
