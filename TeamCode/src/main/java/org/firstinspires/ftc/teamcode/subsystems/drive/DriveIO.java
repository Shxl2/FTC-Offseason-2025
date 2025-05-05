package org.firstinspires.ftc.teamcode.subsystems.drive;

import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

public interface DriveIO {
    public class DriveIOInputs {
        public MecanumDriveWheelPositions wheelPositions = new MecanumDriveWheelPositions();

        public MecanumDriveWheelSpeeds desiredWheelSpeeds = new MecanumDriveWheelSpeeds();
        public MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds();

        public double[] motorVoltages = {0.0};
        public double[] motorCurrents = {0.0};

        private NetworkTable table;
        private StructPublisher<MecanumDriveWheelPositions> wheelPositionsTable;
        private StructPublisher<MecanumDriveWheelSpeeds> desiredWheelSpeedsTable;
        private StructPublisher<MecanumDriveWheelSpeeds> wheelSpeedsTable;

        private DoubleArrayPublisher voltagesTable;
        private DoubleArrayPublisher currentsTable;

        public DriveIOInputs(String tableName) {
            table = NetworkTableInstance.getDefault().getTable("Drive/Localizer");
            wheelPositionsTable = table.getStructTopic("WheelPositions", MecanumDriveWheelPositions.struct).publish();

            desiredWheelSpeedsTable = table.getStructTopic("DesiredWheelSpeeds", MecanumDriveWheelSpeeds.struct).publish();
            wheelSpeedsTable = table.getStructTopic("WheelSpeeds", MecanumDriveWheelSpeeds.struct).publish();

            voltagesTable = table.getDoubleArrayTopic("MotorVoltages").publish();
            currentsTable = table.getDoubleArrayTopic("MotorCurrents").publish();
        }

        public void logValues() {
            wheelPositionsTable.set(wheelPositions);
            desiredWheelSpeedsTable.set(desiredWheelSpeeds);

            wheelSpeedsTable.set(wheelSpeeds);

            voltagesTable.set(motorVoltages);
            currentsTable.set(motorCurrents);
        }
    }

    default void updateInputs(DriveIOInputs inputs) {
    }

    default void setSpeeds(MecanumDriveWheelSpeeds speed) {
    }

    default void setGains(DriveConstants.DriveGains gains) {
    }
}
