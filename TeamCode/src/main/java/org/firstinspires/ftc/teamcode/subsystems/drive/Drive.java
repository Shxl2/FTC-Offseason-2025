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
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    private final DriveIO driveMotors;
    private final LocalizerIO localizer;

    private final DriveIO.DriveIOInputs driveInputs = new DriveIO.DriveIOInputs("Drive/Motors");
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
    private final StructPublisher<Pose2d> poseTable = driveTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> desiredSpeedsTable = driveTable.getStructTopic("DesiredSpeeds", ChassisSpeeds.struct).publish();
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

        kP = new TunableNumber("Drive/kP", gains.kP());
        kI = new TunableNumber("Drive/kI", gains.kI());
        kD = new TunableNumber("Drive/kD", gains.kD());

        kS = new TunableNumber("Drive/kS", gains.kS());
        kV = new TunableNumber("Drive/kV", gains.kV());
        kA = new TunableNumber("Drive/kA", gains.kA());

        maxVelo = new TunableNumber("Drive/maxVelo", 1);
        maxAccel = new TunableNumber("Drive/maxAccel", 1);

        timer = new Timer();
        timer.start();

        driveIO.setGains(gains);
    }

    @Override
    public void periodic() {
        driveMotors.updateInputs(driveInputs);
        driveInputs.logValues();

        localizer.updateInputs(localizerInputs);
        localizerInputs.logValues("Drive/Localizer");

        poseEstimator.updateWithTime(timer.get(), localizerInputs.pose.getRotation(), driveInputs.wheelPositions);

        poseTable.set(getPose());

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
        desiredSpeedsTable.set(speeds);
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
