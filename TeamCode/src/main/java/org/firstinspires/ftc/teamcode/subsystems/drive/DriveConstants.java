package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.Localizers;
import com.pedropathing.localization.constants.OTOSConstants;
import com.pedropathing.localization.constants.PinpointConstants;
import com.pedropathing.localization.localizers.PinpointLocalizer;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DriveConstants {
    public class FConstants {
        static {
            FollowerConstants.localizers = Localizers.PINPOINT;
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

            FollowerConstants.drivePIDFCoefficients.setCoefficients(0.01,0,0.001,0.6,0);
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
            PinpointConstants.forwardY = -1.259;
            PinpointConstants.strafeX = 0.0;
            PinpointConstants.distanceUnit = DistanceUnit.INCH;
            PinpointConstants.hardwareMapName = "pinpoint";
            PinpointConstants.useYawScalar = false;
            PinpointConstants.yawScalar = (double)1.0F;
            PinpointConstants.useCustomEncoderResolution = false;
            PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
            PinpointConstants.customEncoderResolution = 13.26291192;
            PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
            PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        }
    }

}
