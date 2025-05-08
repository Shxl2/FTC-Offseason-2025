package org.firstinspires.ftc.teamcode.lib.ftclib.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

// An OpMode that is has methods similar to WPILib's Robot class
public abstract class TimedRobotOpMode extends OpMode {
    @Override
    public final void init() {
        Robot.disable();
        robotInit();
    }
    @Override
    public final void init_loop() {
        robotPeriodic();
    }

    @Override
    public final void start() {
        Robot.enable();
        enabledInit();
    }

    @Override
    public final void loop() {
        robotPeriodic();
        enabledPeriodic();
    }

    @Override
    public final void stop() {
        disabledInit();
    }

    abstract public void robotInit();

    abstract public void robotPeriodic();

    abstract public void enabledInit();

    abstract public void enabledPeriodic();

    abstract public void disabledInit();
}
