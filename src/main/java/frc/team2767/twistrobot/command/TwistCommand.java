package frc.team2767.twistrobot.command;

import edu.wpi.first.wpilibj.command.Command;
import frc.team2767.twistrobot.Robot;
import frc.team2767.twistrobot.subsystem.DriveSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class TwistCommand extends Command {
  private static final DriveSubsystem drive = Robot.DRIVE;
  private final Logger logger = LoggerFactory.getLogger(this.getClass());
  private double cameraAngle;
  private double cameraRange;

  public TwistCommand(double cameraAngle, double cameraRange) {
    this.cameraRange = cameraRange;
    this.cameraAngle = cameraAngle;

    requires(drive);
  }

  private double computeDirection() {
    // camera angle -> robot direction -> swerve direction (field oriented)
    double currentRobotDirection = drive.getGyroYaw();

    //    double desiredDirection = currentRobotDirection + cameraAngle;
    //    logger.debug("{}", desiredDirection);

    return 0.0;
  }

  private int computeAverageTickCount() {
    int ticks = (int) cameraRange / drive.TICKS_PER_INCH;
    logger.debug("ticks={}", ticks);

    return 0;
  }

  @Override
  protected void initialize() {}

  @Override
  protected void execute() {
    super.execute();
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    super.end();
  }
}
