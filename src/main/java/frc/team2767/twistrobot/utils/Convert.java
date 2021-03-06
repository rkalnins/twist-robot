package frc.team2767.twistrobot.utils;

import frc.team2767.twistrobot.Robot;
import frc.team2767.twistrobot.subsystem.DriveSubsystem;

public class Convert {
  private static final DriveSubsystem DRIVE = Robot.DRIVE;

  public static int inchesToTicks(double inches) {
    return (int) (DRIVE.TICKS_PER_INCH * inches);
  }
}
