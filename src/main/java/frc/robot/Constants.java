/* (C) Robolancers 2024 */
package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

  public static final class DrivetrainConstants {

    public static final double kTrackWidthMeters = Units.inchesToMeters(25.0);
    public static final double kWheelBaseMeters = Units.inchesToMeters(25.0);

    public static final double kMaxSpeedMetersPerSecond = 5.0;
    public static final double kMaxOmegaRadiansPerSecond = 1.5 * Math.PI;

    public static final double kMaxTeleopSpeedPercent = 1.0;
    public static final double kMaxTeleopRotationPercent = 1.0;

    public static final double kTranslationP = 1.8; // 1.15
    public static final double kTranslationI = 0.0;
    public static final double kTranslationD = 0.0; // 0.0

    // corrects heading during path planner
    public static final double kRotationP = 3.0; // 2.16
    public static final double kRotationI = 0.0;
    public static final double kRotationD = 0.25; // 0.0

    // corrects heading during teleop
    public static final double kHeadingP = 0.2; // multiply by 10 if it doesnt explode
    public static final double kHeadingI = 0.0;
    public static final double kHeadingD = 0.01; // 0.01?

    public static final double kHeadingTolerance = 1.5;
  }
}