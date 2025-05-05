package org.firstinspires.ftc.teamcode.subsystems.drive.localizer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

public interface LocalizerIO {
    class LocalizerIOInputs {
        public Pose2d pose = new Pose2d();

        public double accelX = 0.0;
        public double accelY = 0.0;
        public double accelZ = 0.0;

        private final NetworkTable table;
        private final StructPublisher<Pose2d> poseTable;

        private final DoublePublisher accelXTable;
        private final DoublePublisher accelYTable;
        private final DoublePublisher accelZTable;

        public LocalizerIOInputs(String name) {
            table = NetworkTableInstance.getDefault().getTable(name);

            poseTable = table.getStructTopic("Pose", Pose2d.struct).publish();

            accelXTable = table.getDoubleTopic("AccelX").publish();
            accelYTable = table.getDoubleTopic("AccelY").publish();
            accelZTable = table.getDoubleTopic("AccelZ").publish();
        }

        public void logValues() {
            poseTable.set(pose);

            accelXTable.set(accelX);
            accelYTable.set(accelY);
            accelZTable.set(accelZ);
        }
    }

    default void updateInputs(LocalizerIOInputs inputs) {
    }

    default void resetPose(Pose2d pose) {
    }
}
