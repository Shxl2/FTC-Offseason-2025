//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package org.firstinspires.ftc.teamcode.subsystems.drive.localizer;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.SparkFunOTOSCorrected;
import com.pedropathing.localization.constants.OTOSConstants;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class LimelightLocalizerOTOS extends Localizer {
    private HardwareMap hardwareMap;
    private Pose startPose;
    private SparkFunOTOS otos;
    private SparkFunOTOS.Pose2D otosPose;
    private SparkFunOTOS.Pose2D otosVel;
    private SparkFunOTOS.Pose2D otosAcc;
    private double previousHeading;
    private double totalHeading;

    private Limelight3A limelight;

    public LimelightLocalizerOTOS(HardwareMap map) {
        this(map, new Pose());
    }

    public LimelightLocalizerOTOS(HardwareMap map, Pose setStartPose) {
        this.hardwareMap = map;
        if (OTOSConstants.useCorrectedOTOSClass) {
            this.otos = (SparkFunOTOS)this.hardwareMap.get(SparkFunOTOSCorrected.class, OTOSConstants.hardwareMapName);
        } else {
            this.otos = (SparkFunOTOS)this.hardwareMap.get(SparkFunOTOS.class, OTOSConstants.hardwareMapName);
        }

        this.otos.setLinearUnit(OTOSConstants.linearUnit);
        this.otos.setAngularUnit(OTOSConstants.angleUnit);
        this.otos.setOffset(OTOSConstants.offset);
        this.otos.setLinearScalar(OTOSConstants.linearScalar);
        this.otos.setAngularScalar(OTOSConstants.angularScalar);
        this.otos.calibrateImu();
        this.otos.resetTracking();
        this.setStartPose(setStartPose);
        this.otosPose = new SparkFunOTOS.Pose2D();
        this.otosVel = new SparkFunOTOS.Pose2D();
        this.otosAcc = new SparkFunOTOS.Pose2D();
        this.totalHeading = (double)0.0F;
        this.previousHeading = this.startPose.getHeading();
        this.resetOTOS();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();
    }

    public Pose getPose() {
        Pose pose = new Pose(this.otosPose.x, this.otosPose.y, this.otosPose.h);
        Vector vec = pose.getVector();
        vec.rotateVector(this.startPose.getHeading());

        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose llPose = new Pose(result.getBotpose().getPosition().x * 39.37, result.getBotpose().getPosition().y * 39.37, result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS));

//                llPose.subtract(pose);
//                llPose.scalarMultiply(0.1);
//                pose.add(llPose);
                pose = llPose;
                vec = llPose.getVector();
                vec.rotateVector(this.startPose.getHeading());
            }
        }
        return MathFunctions.addPoses(this.startPose, new Pose(vec.getXComponent(), vec.getYComponent(), pose.getHeading()));
    }

    public Pose getVelocity() {
        return new Pose(this.otosVel.x, this.otosVel.y, this.otosVel.h);
    }

    public Vector getVelocityVector() {
        return this.getVelocity().getVector();
    }

    public void setStartPose(Pose setStart) {
        this.startPose = setStart;
    }

    public void setPose(Pose setPose) {
        this.resetOTOS();
        Pose setOTOSPose = MathFunctions.subtractPoses(setPose, this.startPose);
        this.otos.setPosition(new SparkFunOTOS.Pose2D(setOTOSPose.getX(), setOTOSPose.getY(), setOTOSPose.getHeading()));
    }

    public void update() {
        this.otos.getPosVelAcc(this.otosPose, this.otosVel, this.otosAcc);
        this.totalHeading += MathFunctions.getSmallestAngleDifference(this.otosPose.h, this.previousHeading);
        this.previousHeading = this.otosPose.h;
    }

    public void resetOTOS() {
        this.otos.resetTracking();
    }

    public double getTotalHeading() {
        return this.totalHeading;
    }

    public double getForwardMultiplier() {
        return this.otos.getLinearScalar();
    }

    public double getLateralMultiplier() {
        return this.otos.getLinearScalar();
    }

    public double getTurningMultiplier() {
        return this.otos.getAngularScalar();
    }

    public void resetIMU() {
    }

    public boolean isNAN() {
        return Double.isNaN(this.getPose().getX()) || Double.isNaN(this.getPose().getY()) || Double.isNaN(this.getPose().getHeading());
    }
}
