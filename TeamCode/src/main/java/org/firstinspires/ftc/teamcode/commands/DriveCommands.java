package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.lib.controller.SquIDController;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {

    private DriveCommands() {}

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public static Command joystickDrive(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        return Commands.runOnce(drive::startTeleopDrive, drive).andThen(Commands.run(() -> drive.drive(
                xSupplier,
                ySupplier,
                omegaSupplier)));
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotationSupplier) {

        // Create PID controller
        SquIDController angleController =
                new SquIDController(
                        0.1);

        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.runOnce(drive::startTeleopDrive, drive).andThen(
                Commands.run(
                        () -> {
                            // Calculate angular speed
                            double omega =
                                    angleController.calculate(
                                            drive.getPedroPose().getHeading(), rotationSupplier.getAsDouble());

                            drive.drive(
                                    xSupplier,
                                    ySupplier,
                                    () -> omega);
                        },
                        drive));
    }
}
