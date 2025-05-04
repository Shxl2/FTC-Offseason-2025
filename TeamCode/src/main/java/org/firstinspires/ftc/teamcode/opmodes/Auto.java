package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.lib.ftclib.opmode.CommandOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BlueAuto", group = "Auto")
public class Auto extends CommandOpMode {
    private Telemetry m_telemetry;

    @Override
    public void robotInit() {
        m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        new RobotContainer(hardwareMap, m_telemetry, gamepad1, gamepad2, 1); //Uses heavily modified untested hardware
    }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();
        m_telemetry.update();

    }
}