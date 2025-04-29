package org.firstinspires.ftc.teamcode.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public interface VisionIO {
    class VisionIOInputs {
        public TargetObservation latestTargetObservation =
                new TargetObservation(new Rotation2d(), new Rotation2d());
        public PoseObservation poseObservation = new PoseObservation(0, new Pose3d(), 0, 0, 0, PoseObservationType.MEGATAG_1);
        public int[] tagIds = new int[0];

        public void logValues(String name) {
            NetworkTable table = NetworkTableInstance.getDefault().getTable(name);

            table.getStructTopic("TargetObservation/Tx", Rotation2d.struct).publish().set(latestTargetObservation.tx);
            table.getStructTopic("TargetObservation/Ty", Rotation2d.struct).publish().set(latestTargetObservation.ty);
        }
    }

    /** Represents the angle to a simple target, not used for pose estimation. */
    record TargetObservation(Rotation2d tx, Rotation2d ty) {}

    /** Represents a robot pose sample used for pose estimation. */
    record PoseObservation(
            double timestamp,
            Pose3d pose,
            double ambiguity,
            int tagCount,
            double averageTagDistance,
            PoseObservationType type) {}

    enum PoseObservationType {
        MEGATAG_1,
        MEGATAG_2
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}
