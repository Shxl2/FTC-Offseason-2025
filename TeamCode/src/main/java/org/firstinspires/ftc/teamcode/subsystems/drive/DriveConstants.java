package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.pedropathing.localization.constants.OTOSConstants;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DriveConstants {
    public class FConstants {
        static {
            FollowerConstants.localizers = Localizers.OTOS;

            FollowerConstants.leftFrontMotorName = "FL";
            FollowerConstants.leftRearMotorName = "BL";
            FollowerConstants.rightFrontMotorName = "FR";
            FollowerConstants.rightRearMotorName = "BR";

            FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
            FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
            FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
            FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

            FollowerConstants.mass = 13;

            FollowerConstants.xMovement = 57.8741;
            FollowerConstants.yMovement = 52.295;

            FollowerConstants.forwardZeroPowerAcceleration = -41.278;
            FollowerConstants.lateralZeroPowerAcceleration = -59.7819;

            FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0);
            FollowerConstants.useSecondaryTranslationalPID = false;
            FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID

            FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0.1,0);
            FollowerConstants.useSecondaryHeadingPID = false;
            FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID

            FollowerConstants.drivePIDFCoefficients.setCoefficients(0.1,0,0,0.6,0);
            FollowerConstants.useSecondaryDrivePID = false;
            FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1,0,0,0.6,0); // Not being used, @see useSecondaryDrivePID

            FollowerConstants.zeroPowerAccelerationMultiplier = 4;
            FollowerConstants.centripetalScaling = 0.0005;

            FollowerConstants.pathEndTimeoutConstraint = 500;
            FollowerConstants.pathEndTValueConstraint = 0.995;
            FollowerConstants.pathEndVelocityConstraint = 0.1;
            FollowerConstants.pathEndTranslationalConstraint = 0.1;
            FollowerConstants.pathEndHeadingConstraint = 0.007;
        }
    }

    public class LConstants {
        static {
            OTOSConstants.linearUnit = DistanceUnit.INCH;
            OTOSConstants.angleUnit = AngleUnit.RADIANS;
            OTOSConstants.offset = new SparkFunOTOS.Pose2D((double)0.0F, (double)0.0F, (Math.PI / 2D));
            OTOSConstants.linearScalar = (double)1.0F;
            OTOSConstants.angularScalar = (double)1.0F;
            OTOSConstants.hardwareMapName = "sensor_otos";
        }
    }

}
