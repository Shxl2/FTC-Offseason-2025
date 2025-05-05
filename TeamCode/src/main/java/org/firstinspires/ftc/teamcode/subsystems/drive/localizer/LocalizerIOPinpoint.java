package org.firstinspires.ftc.teamcode.subsystems.drive.localizer;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import edu.wpi.first.math.geometry.Pose2d;

public class LocalizerIOPinpoint implements LocalizerIO{
    private final GoBildaPinpointDriver pinpoint;

    public LocalizerIOPinpoint(HardwareMap hwMap) {
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setOffsets(0.0, -1.259, DistanceUnit.INCH);
    }

    @Override
    public void updateInputs(LocalizerIO.LocalizerIOInputs inputs) {
        pinpoint.update();

        inputs.pose = pinpoint.getPosition();
    }

    @Override
    public void resetPose(Pose2d pose) {
        pinpoint.setPosition(pose);
    }
}
