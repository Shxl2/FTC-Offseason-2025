package org.firstinspires.ftc.teamcode.subsystems.drive;

import org.firstinspires.ftc.teamcode.lib.mecanical_advantage.TunableNumber;
import org.firstinspires.ftc.teamcode.lib.wpilib.MecanumDrivePoseEstimator;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizer.LocalizerIO;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    private final DriveIO driveMotors;
    private final LocalizerIO localizer;

    private final DriveIO.DriveIOInputs driveInputs = new DriveIO.DriveIOInputs();
    private final LocalizerIO.LocalizerIOInputs localizerInputs = new LocalizerIO.LocalizerIOInputs();

    private final MecanumDriveKinematics kinematics;
    private final MecanumDrivePoseEstimator poseEstimator;

    private final TunableNumber kP;
    private final TunableNumber kI;
    private final TunableNumber kD;

    private final TunableNumber kS;
    private final TunableNumber kV;
    private final TunableNumber kA;

    private final TunableNumber maxVelo;
    private final TunableNumber maxAccel;

    private final NetworkTable driveTable = NetworkTableInstance.getDefault().getTable("Drive");
    private final Timer timer;

    public Drive(DriveIO driveIO, LocalizerIO localizerIO, DriveConstants.DriveGains gains) {
        driveMotors = driveIO;
        localizer = localizerIO;

        kinematics = new MecanumDriveKinematics(
                DriveConstants.moduleTranslations[0],
                DriveConstants.moduleTranslations[1],
                DriveConstants.moduleTranslations[2],
                DriveConstants.moduleTranslations[3]);

        poseEstimator = new MecanumDrivePoseEstimator(
                kinematics,
                localizerInputs.pose.getRotation(),
                driveInputs.wheelPositions,
                new Pose2d());

        kP = new TunableNumber("Drive/kP", 1);
        kI = new TunableNumber("Drive/kI", 0);
        kD = new TunableNumber("Drive/kD", 0);

        kS = new TunableNumber("Drive/kS", 1);
        kV = new TunableNumber("Drive/kV", 1);
        kA = new TunableNumber("Drive/kA", 0);

        maxVelo = new TunableNumber("Drive/maxVelo", 1);
        maxAccel = new TunableNumber("Drive/maxAccel", 1);

        timer = new Timer();
        timer.start();

        driveIO.setGains(gains);
    }

    @Override
    public void periodic() {
        driveMotors.updateInputs(driveInputs);
        driveInputs.logValues("Drive/Motors");

        localizer.updateInputs(localizerInputs);
        localizerInputs.logValues("Drive/Localizer");

        poseEstimator.updateWithTime(timer.get(), localizerInputs.pose.getRotation(), driveInputs.wheelPositions);

        driveTable.getStructTopic("Drive/Pose", Pose2d.struct).publish().set(getPose());

        TunableNumber.ifChanged(
                hashCode(),
                (values) -> {
                    driveMotors.setGains(
                            new DriveConstants.DriveGains(
                                    values[0],
                                    values[1],
                                    values[2],
                                    values[3],
                                    values[4],
                                    values[5]));
                }, kP, kI, kD, kS, kV, kA);
    }

    public void setSpeeds(ChassisSpeeds speeds) {
        driveTable.getStructTopic("DesiredSpeeds", ChassisSpeeds.struct).publish().set(speeds);
        driveMotors.setSpeeds(kinematics.toWheelSpeeds(speeds));
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRotation() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    public double getMaxVelocity() {
        return maxVelo.get();
    }

    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
         poseEstimator.addVisionMeasurement(
            visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }
}
