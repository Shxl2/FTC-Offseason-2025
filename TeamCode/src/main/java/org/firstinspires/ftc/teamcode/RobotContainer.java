package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DriveCommands;
import org.firstinspires.ftc.teamcode.lib.ftclib.button.GamepadButton;
import org.firstinspires.ftc.teamcode.lib.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.lib.ftclib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.lib.wpilib.CommandGamepad;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveIOEncoders;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizer.LocalizerIOOTOS;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionIOLimelight3A;

import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
    private final Drive drive;
    private final Vision vision;

    private final CommandGamepad driverController;
    private final CommandGamepad operatorController;

    public RobotContainer(HardwareMap hwMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, int autoNum) {
        drive = new Drive(new DriveIOEncoders(hwMap, DriveConstants.driveConfig), new LocalizerIOOTOS(), DriveConstants.driveGains);
        vision = new Vision(drive::addVisionMeasurement, new VisionIOLimelight3A(hwMap));

        driverController = new CommandGamepad(gamepad1);
        operatorController = new CommandGamepad(gamepad2);

        if (autoNum == 0) {
            setDefaultCommands();
            configureButtonBindings();
        }
    }

    public void setDefaultCommands(){
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        driverController::getLeftY,
                        driverController::getLeftX,
                        driverController::getRightX));
    }

    public void configureButtonBindings() {
    }

    public Command getAutoCommand(int chooser) {
        switch (chooser) {
            case 1:
                return null;
            case 2:
                return null;
            case 3:
                return null;
        }
        return null;
    }
}