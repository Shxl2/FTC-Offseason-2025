package org.firstinspires.ftc.teamcode.subsystems.elevator;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.controller.SquIDController;
import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.motors.MotorEx;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final Telemetry telemetry;

    private final MotorEx topMotor;
    private final MotorEx bottomMotor;

    private final SquIDController controller;

    private double kSetpoint;

    private double desiredPosition = 0;

    public Elevator(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        topMotor = new MotorEx(hwMap, "liftTop");
        bottomMotor = new MotorEx(hwMap, "liftBottom");

        topMotor.setInverted(true);
        bottomMotor.setInverted(false);

        topMotor.stopAndResetEncoder();
        bottomMotor.stopAndResetEncoder();

        controller = new SquIDController(0.1);

        kSetpoint = ElevatorConstants.setpoint;
    }

    @Override
    public void periodic() {
        try {
            telemetry.addLine("Elevator");
            telemetry.addData("Elevator Position", topMotor.getCurrentPosition() * ElevatorConstants.ticksToInches);
            telemetry.addData("Elevator Voltage", topMotor.getVoltage());
            telemetry.addData("Elevator Current", topMotor.getCurrent());


            if (kSetpoint != ElevatorConstants.setpoint) {
                kSetpoint = ElevatorConstants.setpoint;
                desiredPosition = kSetpoint;
            }

            calculateVoltage(desiredPosition);
        } catch (Exception e) {

        }
    }

    private void calculateVoltage(double position) {
        telemetry.addData("Elevator Desired Position", position);

        double output = controller.calculate(ElevatorConstants.kP, position, topMotor.getCurrentPosition()) + ElevatorConstants.kG;
        topMotor.set(output);
        bottomMotor.set(output);
    }

    private void setPosition(double position) {
        desiredPosition = position;
    }

    public static Command setPosition(Elevator elevator, DoubleSupplier position) {
        return Commands.run(() -> elevator.setPosition(position.getAsDouble()), elevator);
    }
}
