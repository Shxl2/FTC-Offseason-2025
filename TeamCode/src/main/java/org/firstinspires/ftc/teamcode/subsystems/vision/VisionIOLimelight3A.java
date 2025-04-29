package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.R;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionIOLimelight3A implements VisionIO {
    private final Limelight3A limelight;

    public VisionIOLimelight3A(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();
    }

    @Override
    public void updateInputs(VisionIO.VisionIOInputs inputs) {
        LLResult result = limelight.getLatestResult();

        inputs.latestTargetObservation = new VisionIO.TargetObservation(
                Rotation2d.fromDegrees(result.getTx()),
                Rotation2d.fromDegrees(result.getTy()));

        inputs.poseObservation = new VisionIO.PoseObservation(
                result.getTimestamp(),
                llPoseToWpiPose(result.getBotpose()),
                0.0,
                result.getBotposeTagCount(),
                result.getBotposeAvgDist(),
                VisionIO.PoseObservationType.MEGATAG_1);
    }

    private static Pose3d llPoseToWpiPose(Pose3D pose) {
        return new Pose3d(
                new Translation3d(
                    pose.getPosition().x,
                    pose.getPosition().y,
                    pose.getPosition().z),
                new Rotation3d(
                        pose.getOrientation().getRoll(),
                        pose.getOrientation().getPitch(),
                        pose.getOrientation().getYaw()));
    }
}
