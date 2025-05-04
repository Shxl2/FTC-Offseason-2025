package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.localizers.OTOSLocalizer;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizer.LimelightLocalizerOTOS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    private final Follower drive;
    private final Telemetry telemetry;

    public Drive(HardwareMap hwMap, Telemetry telemetry) {
        drive = new Follower(hwMap, DriveConstants.FConstants.class, DriveConstants.LConstants.class);

//        drive.initialize(new LimelightLocalizerOTOS(hwMap));
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        try {
            drive.update();
            drive.drawOnDashBoard();
            drive.telemetryDebug(telemetry);
        } catch (Exception ignored) {
        }
    }

    // TODO: Make this reflect speed properly
    public void setSpeeds(ChassisSpeeds speeds) {
        drive.startTeleopDrive();
        drive.setTeleOpMovementVectors(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, true);
    }

    public Pose2d getPose() {
        return new Pose2d(new Translation2d(drive.getPose().getX(), drive.getPose().getY()), new Rotation2d(drive.getPose().getHeading()));
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public double getMaxVelocity() {
        return 1.0;
    }

    public PathBuilder getPathBuilder() {
        return drive.pathBuilder();
    }

    public void followPath(PathChain path) {
        drive.followPath(path, false);
    }

    public boolean isFinished() {
        return drive.atParametricEnd();
    }

}