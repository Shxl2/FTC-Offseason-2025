package org.firstinspires.ftc.teamcode.subsystems.vision;

import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.angularStdDevBaseline;
import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.angularStdDevMegatag2Factor;
import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.linearStdDevBaseline;
import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.linearStdDevMegatag2Factor;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    private final Limelight3A limelight;

    public Vision(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");

        limelight.start();
    }


    @Override
    public void periodic() {
        LLResult result = limelight.getLatestResult();

    }

}
