package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.lib.wpilib.TunableSimpleMotorFeedforward;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;

public class DriveIOEncoders implements DriveIO {
    double rotationsToMeters;

    private final MotorEx fL;
    private final MotorEx fR;
    private final MotorEx bL;
    private final MotorEx bR;

    private TunableSimpleMotorFeedforward feedforward;

    private final PIDController fLController;
    private final PIDController fRController;
    private final PIDController bLController;
    private final PIDController bRController;

    public DriveIOEncoders(HardwareMap hwMap, DriveConstants.DriveConfig config) {
        rotationsToMeters = (config.wheelRadius() * 2 * Math.PI) / config.gearRatio();

        fL = new MotorEx(hwMap, "FL");
        fR = new MotorEx(hwMap, "FR");
        bL = new MotorEx(hwMap, "BL");
        bR = new MotorEx(hwMap, "BR");

        feedforward = new TunableSimpleMotorFeedforward(0, 0, 0);

        fLController = new PIDController(0, 0, 0);
        fRController = new PIDController(0, 0, 0);
        bLController = new PIDController(0, 0, 0);
        bRController = new PIDController(0, 0, 0);
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        inputs.wheelPositions = getPositions();
        inputs.wheelSpeeds = getSpeeds();

        inputs.motorVoltages = new double[] {fL.getVoltage(), fR.getVoltage(), bL.getVoltage(), bR.getVoltage()};
        inputs.motorCurrents = new double[] {fL.getCurrent(), fR.getCurrent(), bL.getCurrent(), bR.getCurrent()};
    }

    @Override
    public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
        double fLFeedback = fLController.calculate(getSpeeds().frontLeftMetersPerSecond, speeds.frontLeftMetersPerSecond);
        double fRFeedback = fRController.calculate(getSpeeds().frontRightMetersPerSecond, speeds.frontRightMetersPerSecond);
        double bLFeedback = bLController.calculate(getSpeeds().rearLeftMetersPerSecond, speeds.rearLeftMetersPerSecond);
        double bRFeedback = bRController.calculate(getSpeeds().rearRightMetersPerSecond, speeds.rearRightMetersPerSecond);

        double fLFF = feedforward.calculateWithVelocities(getSpeeds().frontLeftMetersPerSecond, speeds.frontLeftMetersPerSecond);
        double fRFF = feedforward.calculateWithVelocities(getSpeeds().frontRightMetersPerSecond, speeds.frontRightMetersPerSecond);
        double bLFF = feedforward.calculateWithVelocities(getSpeeds().rearLeftMetersPerSecond, speeds.rearLeftMetersPerSecond);
        double bRFF = feedforward.calculateWithVelocities(getSpeeds().rearRightMetersPerSecond, speeds.rearLeftMetersPerSecond);

        fL.setVoltage(fLFeedback + fLFF);
        fR.setVoltage(fRFeedback + fRFF);
        bL.setVoltage(bLFeedback + bLFF);
        bR.setVoltage(bRFeedback + bRFF);
    }

    @Override
    public void setGains(DriveConstants.DriveGains gains) {
        fLController.setPID(gains.kP(), gains.kI(), gains.kD());
        fRController.setPID(gains.kP(), gains.kI(), gains.kD());
        bLController.setPID(gains.kP(), gains.kI(), gains.kD());
        bRController.setPID(gains.kP(), gains.kI(), gains.kD());

        feedforward.setGains(gains.kS(), gains.kV(), gains.kA());
    }

    private MecanumDriveWheelPositions getPositions() {
        return new MecanumDriveWheelPositions(
                fL.getRevolutions() * rotationsToMeters,
                fR.getRevolutions() * rotationsToMeters,
                bL.getRevolutions() * rotationsToMeters,
                bR.getRevolutions() * rotationsToMeters);
    }

    private MecanumDriveWheelSpeeds getSpeeds() {
        return new MecanumDriveWheelSpeeds(fL.getVelocity(), fR.getVelocity(), bL.getVelocity(), bR.getVelocity());
    }
}
