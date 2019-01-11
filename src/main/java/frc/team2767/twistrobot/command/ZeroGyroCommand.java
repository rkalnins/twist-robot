package frc.team2767.twistrobot.command;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team2767.twistrobot.Robot;
import frc.team2767.twistrobot.subsystem.DriveSubsystem;

public final class ZeroGyroCommand extends InstantCommand {

  private static final DriveSubsystem swerve = Robot.DRIVE;

  public ZeroGyroCommand() {
    requires(swerve);
  }

  @Override
  protected void initialize() {
    swerve.zeroGyro();
  }
}
