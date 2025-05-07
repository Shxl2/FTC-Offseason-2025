package org.firstinspires.ftc.teamcode.lib.controller;

import edu.wpi.first.math.MathUtil;

public class SquIDController {
    private double squP;

    private boolean continuous;
    private double maximumInput;
    private double minimumInput;

    public SquIDController(double squP) {
        this.squP = squP;
    }

    public double calculate(double setpoint, double measurement) {
        double error;
        if (continuous) {
            double errorBound = (maximumInput - minimumInput) / 2.0;
            error = MathUtil.inputModulus(setpoint - measurement, -errorBound, errorBound);
        } else {
            error = setpoint - measurement;
        }

        return squP * Math.sqrt(Math.abs(error)) * Math.signum(error);
    }

    public void enableContinuousInput(double minimumInput, double maximumInput) {
        continuous = true;

        this.minimumInput = minimumInput;
        this.maximumInput = maximumInput;
    }
}
