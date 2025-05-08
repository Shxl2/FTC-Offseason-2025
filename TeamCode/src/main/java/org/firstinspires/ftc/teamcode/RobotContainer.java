package org.firstinspires.ftc.teamcode;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AutoCommands;
import org.firstinspires.ftc.teamcode.commands.DriveCommands;
import org.firstinspires.ftc.teamcode.lib.wpilib.CommandGamepad;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class RobotContainer {
//    private final Drive drive;
    private final Elevator elevator;

    private final CommandGamepad driverController;
    private final CommandGamepad operatorController;

    public RobotContainer(HardwareMap hwMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, int autoNum) {
//        drive = new Drive(hwMap, telemetry);
        elevator = new Elevator(hwMap, telemetry);

        driverController = new CommandGamepad(gamepad1);
        operatorController = new CommandGamepad(gamepad2);

        if (autoNum == 0) {
            setDefaultCommands();
            configureButtonBindings();
        } else {
            getAutoCommand(autoNum);
        }
    }

    public void setDefaultCommands(){
//        drive.setDefaultCommand(
//                DriveCommands.joystickDrive(
//                        drive,
//                        () -> -driverController.getLeftY(),
//                        () -> -driverController.getLeftX(),
//                        () -> -driverController.getRightX()));
    }

    public void configureButtonBindings() {
//        driverController.a().whileTrue(DriveCommands.joystickDriveAtAngle(drive, () -> -driverController.getLeftY(),
//                () -> -driverController.getLeftX(), () -> Math.toRadians(90)));
    }

    public Command getAutoCommand(int chooser) {
        switch (chooser) {
            case 1:
//                return AutoCommands.bucketAuto(drive);
        }
        return Commands.none();
    }
}