package org.firstinspires.ftc.teamcode.subsystems.elevator;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.controller.SquIDController;
import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.motors.MotorEx;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final Telemetry telemetry;

    private final MotorEx topMotor;
    private final MotorEx bottomMotor;

    private final SquIDController controller;

    public Elevator(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        topMotor = new MotorEx(hwMap, "liftTop");
        bottomMotor = new MotorEx(hwMap, "liftBottom");

        topMotor.setInverted(true);
        bottomMotor.setInverted(true);

        controller = new SquIDController(0.1);
    }

    @Override
    public void periodic() {
        try {
            telemetry.addLine("Elevator");
            telemetry.addData("Elevator Position", topMotor.getCurrentPosition() * ElevatorConstants.ticksToInches);

            setPosition(ElevatorConstants.setpoint);
        } catch (Exception e) {

        }

    }

    public void setPosition(double position) {
        double output = controller.calculate(ElevatorConstants.kP, topMotor.getCurrentPosition(), position);
        topMotor.set(output);
        bottomMotor.set(output);
    }
}
