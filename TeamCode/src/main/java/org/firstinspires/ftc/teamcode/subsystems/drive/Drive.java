package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.mecanical_advantage.TunableNumber;
import org.firstinspires.ftc.teamcode.lib.pathplanner.AdvancedPPHolonomicDriveController;
import org.firstinspires.ftc.teamcode.lib.wpilib.MecanumDrivePoseEstimator;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizer.LocalizerIO;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    private final DriveIO driveMotors;
    private final LocalizerIO localizer;

    private final DriveIO.DriveIOInputs driveInputs = new DriveIO.DriveIOInputs("Drive/Motors");
    private final LocalizerIO.LocalizerIOInputs localizerInputs = new LocalizerIO.LocalizerIOInputs("Drive/Localizer");

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
    private final StructPublisher<ChassisSpeeds> actualSpeedsTable = driveTable.getStructTopic("ActualSpeeds", ChassisSpeeds.struct).publish();
    private final Timer timer;

    public Drive(HardwareMap hwMap, DriveIO driveIO, LocalizerIO localizerIO, DriveConstants.DriveGains gains) {
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

        RobotConfig config = RobotConfig.fromGUISettings(hwMap);

        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getSpeeds,
                this::setSpeeds,
                new AdvancedPPHolonomicDriveController(
                        new PIDConstants(DriveConstants.TRANSLATION_P, DriveConstants.TRANSLATION_I, DriveConstants.TRANSLATION_D),
                        new PIDConstants(DriveConstants.HEADING_P, DriveConstants.HEADING_I, DriveConstants.HEADING_D)
                ),
                new RobotConfig(1, 1, new ModuleConfig(0.1, 2, 0.5, DCMotor.getCIM(1), 20, 1), DriveConstants.moduleTranslations),
                () -> {
//                        // Boolean supplier that controls when the path will be mirrored for the red alliance
//                        // This will flip the path being followed to the red side of the field.
//                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
//
//                        var alliance = DriverStation.getAlliance();
//                        if (alliance.isPresent()) {
//                            return alliance.get() == DriverStation.Alliance.Red;
//                        }
//                        return false;
                    return false;
                },
                this
        );
    }

    @Override
    public void periodic() {
        driveMotors.updateInputs(driveInputs);
        driveInputs.logValues();

        localizer.updateInputs(localizerInputs);
        localizerInputs.logValues();

        poseEstimator.updateWithTime(timer.get(), localizerInputs.pose.getRotation(), driveInputs.wheelPositions);

        poseTable.set(getPose());

        System.out.println(timer.get());

        actualSpeedsTable.set(getSpeeds());
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

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(driveInputs.wheelSpeeds);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        localizer.resetPose(pose);
        poseEstimator.resetPosition(localizerInputs.pose.getRotation(), new MecanumDriveWheelPositions(), pose);
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
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
