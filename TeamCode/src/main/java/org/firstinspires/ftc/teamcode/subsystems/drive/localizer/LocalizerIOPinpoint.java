package org.firstinspires.ftc.teamcode.subsystems.drive.localizer;

import com.qualcomm.robotcore.hardware.HardwareMap;

import edu.wpi.first.math.geometry.Pose2d;

public class LocalizerIOPinpoint implements LocalizerIO{
    private final GoBildaPinpointDriver pinpoint;

    public LocalizerIOPinpoint(HardwareMap hwMap) {
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
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
