package org.firstinspires.ftc.teamcode.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public record DriveConfig(double gearRatio, double wheelRadius) {}

    public record DriveGains(double kP, double kI, double kD, double kS, double kV, double kA) {}

    public static double wheelBase = Units.inchesToMeters(7.68);
    public static double trackWidth = Units.inchesToMeters(9.45);

    public static final Translation2d[] moduleTranslations =
            new Translation2d[] {
                    new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
                    new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
                    new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
                    new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
            };

    public static DriveGains driveGains = new DriveGains(0.5, 0, 0, 0.35, 6.4, 0);
    public static DriveConfig driveConfig = new DriveConfig(19.0, (104.0 / 2.0) * 0.001);
}
