package org.firstinspires.ftc.teamcode.subsystems.drive;

import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public interface DriveIO {
    class DriveIOInputs {
        public MecanumDriveWheelPositions wheelPositions = new MecanumDriveWheelPositions();

        public MecanumDriveWheelSpeeds desiredWheelSpeeds = new MecanumDriveWheelSpeeds();
        public MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds();

        public double[] motorVoltages = {0.0};
        public double[] motorCurrents = {0.0};

        public void logValues(String name) {
            NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
            table.getStructTopic("WheelPositions", MecanumDriveWheelPositions.struct).publish().set(wheelPositions);

            table.getStructTopic("DesiredWheelSpeeds", MecanumDriveWheelSpeeds.struct).publish().set(desiredWheelSpeeds);
            table.getStructTopic("WheelSpeeds", MecanumDriveWheelSpeeds.struct).publish().set(wheelSpeeds);

            table.getDoubleArrayTopic("MotorVoltages").publish().set(motorVoltages);
            table.getDoubleArrayTopic("MotorCurrents").publish().set(motorCurrents);
        }
    }

    default void updateInputs(DriveIOInputs inputs) {
    }

    default void setSpeeds(MecanumDriveWheelSpeeds speed) {
    }

    default void setGains(DriveConstants.DriveGains gains) {
    }
}
