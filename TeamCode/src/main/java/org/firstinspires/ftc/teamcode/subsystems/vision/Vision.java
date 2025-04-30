package org.firstinspires.ftc.teamcode.subsystems.vision;

import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.angularStdDevBaseline;
import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.angularStdDevMegatag2Factor;
import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.linearStdDevBaseline;
import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.linearStdDevMegatag2Factor;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final VisionConsumer consumer;
    private final VisionIO io;
    private final VisionIO.VisionIOInputs inputs;

    private final NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("Vision");

    public Vision(VisionConsumer consumer, VisionIO io) {
        this.consumer = consumer;
        this.io = io;

        // Initialize inputs
        this.inputs = new VisionIO.VisionIOInputs();
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing with vision.
     *
     */
    public Rotation2d getTargetX() {
        return inputs.latestTargetObservation.tx();
    }

    @Override
    public void periodic() {
       io.updateInputs(inputs);
       inputs.logValues("Vision");

        // Initialize logging values
        Pose3d robotPose;
        var observation = inputs.poseObservation;
        robotPose = observation.pose();

        // Calculate standard deviations
        double stdDevFactor =
                Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == VisionIO.PoseObservationType.MEGATAG_2) {
            linearStdDev *= linearStdDevMegatag2Factor;
            angularStdDev *= angularStdDevMegatag2Factor;
        }

        // Send vision observation
        consumer.accept(
                observation.pose().toPose2d(),
                observation.timestamp(),
                VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));


        visionTable.getStructTopic("Camera/RobotPose", Pose3d.struct).publish().set(robotPose);
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(
                Pose2d visionRobotPoseMeters,
                double timestampSeconds,
                Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
