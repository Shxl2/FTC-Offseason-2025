package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.commands.auto.BucketAutoPaths;
import org.firstinspires.ftc.teamcode.commands.auto.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

@Config
public class AutoCommands {
    private static final Pose startPose = new Pose(0,0, Math.toRadians(0));
    private static final Pose interPose = new Pose(10, 0, Math.toRadians(0));
    private static final Pose endPose = new Pose(24, 24, Math.toRadians(45));

    public static PathChain sampleAutoPath(Drive drive) {
        // Create a path chain with the desired points
        PathChain path = drive.getPathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(interPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), interPose.getHeading())
//                .addPath(new BezierLine(new Point(interPose), new Point(endPose)))
//                .setLinearHeadingInterpolation(interPose.getHeading(), endPose.getHeading())
//                .addPath(new BezierLine(new Point(endPose), new Point(startPose)))
//                .setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading())
                .build();

        return path;
    }

    public static Command followPath(Drive drive, PathChain chain) {
        return Commands.runOnce(() -> drive.followPath(chain), drive);
    }

    public static Command driveToPose(Drive drive, Pose pose) {
        return Commands.runOnce(() -> drive.followPath(
                drive.getPathBuilder()
                        .addPath(new BezierLine(new Point(drive.getPedroPose()), new Point(pose)))
                        .setLinearHeadingInterpolation(drive.getPedroPose().getHeading(), pose.getHeading())
                        .build()), drive);
    }

    public static Command driveToPose(Drive drive, Pose startPose, Pose endPose) {
        return Commands.runOnce(() -> {
            drive.setPose(startPose);
            drive.followPath(
                drive.getPathBuilder()
                        .addPath(new BezierLine(new Point(startPose), new Point(endPose)))
                        .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                        .build());
            }, drive).until(drive::isFinished);
    }

    public static Command bucketAuto(Drive drive) {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> drive.setPose(new Pose(8.000, 112.000, Math.toRadians(90))), drive),
                AutoCommands.followPath(drive, BucketAutoPaths.line1).andThen(Commands.runOnce(() -> PoseStorage.currentPose = drive.getPedroPose()))
        );
    }
}
