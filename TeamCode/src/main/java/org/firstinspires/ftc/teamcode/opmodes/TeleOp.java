package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.lib.ftclib.opmode.CommandOpMode;

import edu.wpi.first.wpilibj.Timer;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "BLUETeleOp", group = "TeleOp")
public class TeleOp extends CommandOpMode {
    private Telemetry robotTelemetry;
    private Timer timer = new Timer();

    private double previousTime;

    @Override
    public void robotInit() {
        robotTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        new RobotContainer(hardwareMap, robotTelemetry, gamepad1, gamepad2, 0); //Uses heavily modified untested hardware
        timer.start();
    }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();
        robotTelemetry.addData("Loop Time", 1.0 / (timer.get() - previousTime));

        robotTelemetry.update();


        previousTime = timer.get();

    }
}