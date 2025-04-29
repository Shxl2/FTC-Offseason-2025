package org.firstinspires.ftc.teamcode.subsystems.drive.localizer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public interface LocalizerIO {
    class LocalizerIOInputs {
        public Pose2d pose = new Pose2d();

        public double accelX = 0.0;
        public double accelY = 0.0;
        public double accelZ = 0.0;

        public void logValues(String name) {
            NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
            table.getStructTopic("Pose", Pose2d.struct).publish().set(pose);

            table.getDoubleTopic("AccelX").publish().set(accelX);
            table.getDoubleTopic("AccelY").publish().set(accelY);
            table.getDoubleTopic("AccelZ").publish().set(accelZ);
        }
    }

    default void updateInputs(LocalizerIOInputs inputs) {
    }

    default void resetPose(Pose2d pose) {
    }
}
