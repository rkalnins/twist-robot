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
  private static final DriveSubsystem DRIVE = Robot.DRIVE;
  private static final DriverControls CONTROLS = Robot.CONTROLS.getDriverControls();
  private final Logger logger = LoggerFactory.getLogger(this.getClass());

  public TeleOpDriveCommand() {
    requires(DRIVE);
  }

  @Override
  protected void initialize() {
    DRIVE.setDriveMode(TELEOP);
  }

  @Override
  protected void execute() {

    double forward = deadband(CONTROLS.getForward());
    double strafe = deadband(CONTROLS.getStrafe());
    double yaw = deadband(CONTROLS.getYaw());
    //    double yaw = 0.0; // not used

    DRIVE.drive(forward, strafe, yaw);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    DRIVE.stop();
  }

  private double deadband(double value) {
    if (Math.abs(value) < DEADBAND) return 0.0;
    return value;
  }
}
