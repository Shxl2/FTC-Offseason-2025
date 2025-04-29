package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "camera_0";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 =
            new Transform3d(0.0, 0.2, 0.0, new Rotation3d(0.0, 0, Math.PI / 2));
    public static Transform3d robotToCamera1 =
            new Transform3d(0.1, -0.22, 0.2, new Rotation3d(0.0, 0.0, 0.0));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 10;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
            new double[] {
                    1.0, // Camera 0
                    1.0 // Camera 1
            };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
            Double.POSITIVE_INFINITY; // No rotation data available
}
