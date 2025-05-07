package org.firstinspires.ftc.teamcode.commands.auto;

import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class BucketAutoPaths {

    public static PathBuilder builder = new PathBuilder();

    public static PathChain line1 = builder
            .addPath(
                    new BezierLine(
                            new Point(8.000, 112.000, Point.CARTESIAN),
                            new Point(8.000, 124.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
            .build();

    public static PathChain line2 = builder
            .addPath(
                    new BezierLine(
                            new Point(8.000, 124.000, Point.CARTESIAN),
                            new Point(42.142, 120.626, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
            .build();

    public static PathChain line3 = builder
            .addPath(
                    new BezierLine(
                            new Point(42.142, 120.626, Point.CARTESIAN),
                            new Point(13.000, 130.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
            .build();
}


