package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

@Config
public class AutoCommands {
    private static final Pose startPose = new Pose(0,0, Math.toRadians(0));
    private static final Pose interPose = new Pose(24, -24, Math.toRadians(90));
    private static final Pose endPose = new Pose(24, 24, Math.toRadians(45));

    public static PathChain sampleAutoPath(Drive drive) {
        // Create a path chain with the desired points
        PathChain path = drive.getPathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(interPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), interPose.getHeading())
                .addPath(new BezierLine(new Point(interPose), new Point(endPose)))
                .setLinearHeadingInterpolation(interPose.getHeading(), endPose.getHeading())
                .addPath(new BezierLine(new Point(endPose), new Point(startPose)))
                .setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading())
                .build();

        return path;
    }

    public static Command followPath(Drive drive, PathChain chain) {
        return Commands.run(() -> drive.followPath(chain), drive);
    }
}
