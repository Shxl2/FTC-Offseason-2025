package com.pathplanner.lib.config;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ejml.simple.SimpleMatrix;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

/**
 * Configuration class describing everything that needs to be known about the robot to generate
 * trajectories
 */
public class RobotConfig {
  /** The mass of the robot, including bumpers and battery, in KG */
  public final double massKG;
  /** The moment of inertia of the robot, in KG*M^2 */
  public final double MOI;
  /** The drive module config */
  public final ModuleConfig moduleConfig;

  /** Robot-relative locations of each drive module in meters */
  public final Translation2d[] moduleLocations;
  /** Is the robot holonomic? */
  public final boolean isHolonomic;

  private final SwerveDriveKinematics swerveKinematics;
  private final DifferentialDriveKinematics diffKinematics;
  private final SimpleMatrix forceKinematics;

  // Pre-calculated values that can be reused for every trajectory generation
  /** Number of drive modules */
  public final int numModules;
  /** The distance from the robot center to each module in meters */
  public final double[] modulePivotDistance;
  /** The force of static friction between the robot's drive wheels and the carpet, in Newtons */
  public final double wheelFrictionForce;
  /** The maximum torque a drive module can apply without slipping the wheels */
  public final double maxTorqueFriction;

  /**
   * Create a robot config object for a HOLONOMIC DRIVE robot
   *
   * @param massKG The mass of the robot, including bumpers and battery, in KG
   * @param MOI The moment of inertia of the robot, in KG*M^2
   * @param moduleConfig The drive module config
   * @param moduleOffsets The locations of the module relative to the physical center of the robot.
   *     Only robots with 4 modules are supported, and they should be in FL, FR, BL, BR order.
   */
  public RobotConfig(
      double massKG, double MOI, ModuleConfig moduleConfig, Translation2d... moduleOffsets) {
    this.massKG = massKG;
    this.MOI = MOI;
    this.moduleConfig = moduleConfig;

    if (moduleOffsets.length != 4) {
      throw new IllegalArgumentException(
          "PathPlannerLib currently only supports using 4 swerve modules");
    }
    this.moduleLocations = moduleOffsets;
    this.swerveKinematics = new SwerveDriveKinematics(this.moduleLocations);
    this.diffKinematics = null;
    this.isHolonomic = true;

    this.numModules = this.moduleLocations.length;
    this.modulePivotDistance = new double[this.numModules];
    for (int i = 0; i < this.numModules; i++) {
      this.modulePivotDistance[i] = this.moduleLocations[i].getNorm();
    }
    this.wheelFrictionForce = this.moduleConfig.wheelCOF * ((this.massKG / numModules) * 9.8);
    this.maxTorqueFriction = this.wheelFrictionForce * this.moduleConfig.wheelRadiusMeters;

    this.forceKinematics = new SimpleMatrix(this.numModules * 2, 3);
    for (int i = 0; i < this.numModules; i++) {
      Translation2d modPosReciprocal =
          new Translation2d(
              1.0 / this.moduleLocations[i].getNorm(), this.moduleLocations[i].getAngle());
      this.forceKinematics.setRow(i * 2, 0, /* Start Data */ 1, 0, -modPosReciprocal.getY());
      this.forceKinematics.setRow(i * 2 + 1, 0, /* Start Data */ 0, 1, modPosReciprocal.getX());
    }
  }

  /**
   * Create a robot config object for a HOLONOMIC DRIVE robot
   *
   * @param mass The mass of the robot, including bumpers and battery
   * @param MOI The moment of inertia of the robot
   * @param moduleConfig The drive module config
   * @param moduleOffsets The locations of the module relative to the physical center of the robot.
   *     Only robots with 4 modules are supported, and they should be in FL, FR, BL, BR order.
   */
  public RobotConfig(
      Mass mass, MomentOfInertia MOI, ModuleConfig moduleConfig, Translation2d... moduleOffsets) {
    this(mass.in(Kilograms), MOI.in(KilogramSquareMeters), moduleConfig, moduleOffsets);
  }

  /**
   * Create a robot config object for a DIFFERENTIAL DRIVE robot
   *
   * @param massKG The mass of the robot, including bumpers and battery, in KG
   * @param MOI The moment of inertia of the robot, in KG*M^2
   * @param moduleConfig The drive module config
   * @param trackwidthMeters The distance between the left and right side of the drivetrain, in
   *     meters
   */
  public RobotConfig(
      double massKG, double MOI, ModuleConfig moduleConfig, double trackwidthMeters) {
    this.massKG = massKG;
    this.MOI = MOI;
    this.moduleConfig = moduleConfig;

    this.moduleLocations =
        new Translation2d[] {
          new Translation2d(0.0, trackwidthMeters / 2.0),
          new Translation2d(0.0, -trackwidthMeters / 2.0),
        };
    this.swerveKinematics = null;
    this.diffKinematics = new DifferentialDriveKinematics(trackwidthMeters);
    this.isHolonomic = false;

    this.numModules = this.moduleLocations.length;
    this.modulePivotDistance = new double[this.numModules];
    for (int i = 0; i < this.numModules; i++) {
      this.modulePivotDistance[i] = this.moduleLocations[i].getNorm();
    }
    this.wheelFrictionForce = this.moduleConfig.wheelCOF * ((this.massKG / numModules) * 9.8);
    this.maxTorqueFriction = this.wheelFrictionForce * this.moduleConfig.wheelRadiusMeters;

    this.forceKinematics = new SimpleMatrix(this.numModules * 2, 3);
    for (int i = 0; i < this.numModules; i++) {
      Translation2d modPosReciprocal =
          new Translation2d(
              1.0 / this.moduleLocations[i].getNorm(), this.moduleLocations[i].getAngle());
      this.forceKinematics.setRow(i * 2, 0, /* Start Data */ 1, 0, -modPosReciprocal.getY());
      this.forceKinematics.setRow(i * 2 + 1, 0, /* Start Data */ 0, 1, modPosReciprocal.getX());
    }
  }

  /**
   * Create a robot config object for a DIFFERENTIAL DRIVE robot
   *
   * @param mass The mass of the robot, including bumpers and battery
   * @param MOI The moment of inertia of the robot
   * @param moduleConfig The drive module config
   * @param trackwidthMeters The distance between the left and right side of the drivetrain
   */
  public RobotConfig(
      Mass mass, MomentOfInertia MOI, ModuleConfig moduleConfig, Distance trackwidthMeters) {
    this(
        mass.in(Kilograms),
        MOI.in(KilogramSquareMeters),
        moduleConfig,
        trackwidthMeters.in(Meters));
  }

  /**
   * Convert robot-relative chassis speeds to an array of swerve module states. This will use
   * differential kinematics for diff drive robots, then convert the wheel speeds to module states.
   *
   * @param speeds Robot-relative chassis speeds
   * @return Array of swerve module states
   */
  public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds speeds) {
    if (isHolonomic) {
      return swerveKinematics.toSwerveModuleStates(speeds);
    } else {
      var wheelSpeeds = diffKinematics.toWheelSpeeds(speeds);
      return new SwerveModuleState[] {
        new SwerveModuleState(wheelSpeeds.leftMetersPerSecond, new Rotation2d()),
        new SwerveModuleState(wheelSpeeds.rightMetersPerSecond, new Rotation2d())
      };
    }
  }

  /**
   * Convert an array of swerve module states to robot-relative chassis speeds. This will use
   * differential kinematics for diff drive robots.
   *
   * @param states Array of swerve module states
   * @return Robot-relative chassis speeds
   */
  public ChassisSpeeds toChassisSpeeds(SwerveModuleState[] states) {
    if (isHolonomic) {
      return swerveKinematics.toChassisSpeeds(states);
    } else {
      var wheelSpeeds =
          new DifferentialDriveWheelSpeeds(
              states[0].speedMetersPerSecond, states[1].speedMetersPerSecond);
      return diffKinematics.toChassisSpeeds(wheelSpeeds);
    }
  }

  /**
   * Convert chassis forces (passed as ChassisSpeeds) to individual wheel force vectors
   *
   * @param chassisForces The linear X/Y force and torque acting on the whole robot
   * @return Array of individual wheel force vectors
   */
  public Translation2d[] chassisForcesToWheelForceVectors(ChassisSpeeds chassisForces) {
    var chassisForceVector = new SimpleMatrix(3, 1);
    chassisForceVector.setColumn(
        0,
        0,
        chassisForces.vxMetersPerSecond,
        chassisForces.vyMetersPerSecond,
        chassisForces.omegaRadiansPerSecond);

    // Divide the chassis force vector by numModules since force is additive. All module forces will
    // add up to the chassis force
    var moduleForceMatrix = forceKinematics.mult(chassisForceVector.divide(numModules));

    Translation2d[] forceVectors = new Translation2d[numModules];
    for (int m = 0; m < numModules; m++) {
      double x = moduleForceMatrix.get(m * 2, 0);
      double y = moduleForceMatrix.get(m * 2 + 1, 0);

      forceVectors[m] = new Translation2d(x, y);
    }

    return forceVectors;
  }

  /**
   * Load the robot config from the shared settings file created by the GUI
   *
   * @return RobotConfig matching the robot settings in the GUI
   * @throws IOException if an I/O error occurs
   * @throws ParseException if a JSON parsing error occurs
   */
  public static RobotConfig fromGUISettings(HardwareMap hwMap) {
      BufferedReader br =
              null;
      try {
          br = new BufferedReader(
                  new InputStreamReader(
                          hwMap.appContext
                                  .getAssets()
                                  .open("pathplanner/settings.json")));
      } catch (IOException e) {
          throw new RuntimeException(e);
      }

      StringBuilder fileContentBuilder = new StringBuilder();
    String line;
    while (true) {
        try {
            if (!((line = br.readLine()) != null)) break;
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        fileContentBuilder.append(line);
    }
      try {
          br.close();
      } catch (IOException e) {
          throw new RuntimeException(e);
      }

      String fileContent = fileContentBuilder.toString();
      JSONObject json = null;
      try {
          json = (JSONObject) new JSONParser().parse(fileContent);
      } catch (ParseException e) {
          throw new RuntimeException(e);
      }

      boolean isHolonomic = (boolean) json.get("holonomicMode");
    double massKG = ((Number) json.get("robotMass")).doubleValue();
    double MOI = ((Number) json.get("robotMOI")).doubleValue();
    double wheelRadius = ((Number) json.get("driveWheelRadius")).doubleValue();
    double gearing = ((Number) json.get("driveGearing")).doubleValue();
    double maxDriveSpeed = ((Number) json.get("maxDriveSpeed")).doubleValue();
    double wheelCOF = ((Number) json.get("wheelCOF")).doubleValue();
    String driveMotor = (String) json.get("driveMotorType");
    double driveCurrentLimit = ((Number) json.get("driveCurrentLimit")).doubleValue();

    int numMotors = isHolonomic ? 1 : 2;
    DCMotor gearbox =
        switch (driveMotor) {
          case "krakenX60" -> DCMotor.getKrakenX60(numMotors);
          case "krakenX60FOC" -> DCMotor.getKrakenX60Foc(numMotors);
          case "falcon500" -> DCMotor.getFalcon500(numMotors);
          case "falcon500FOC" -> DCMotor.getFalcon500Foc(numMotors);
          case "vortex" -> DCMotor.getNeoVortex(numMotors);
          case "NEO" -> DCMotor.getNEO(numMotors);
          case "CIM" -> DCMotor.getCIM(numMotors);
          case "miniCIM" -> DCMotor.getMiniCIM(numMotors);
          default -> throw new IllegalArgumentException("Invalid motor type: " + driveMotor);
        };
    gearbox = gearbox.withReduction(gearing);

    ModuleConfig moduleConfig =
        new ModuleConfig(
            wheelRadius, maxDriveSpeed, wheelCOF, gearbox, driveCurrentLimit, numMotors);

    if (isHolonomic) {
      Translation2d[] moduleOffsets =
          new Translation2d[] {
            new Translation2d(
                ((Number) json.get("flModuleX")).doubleValue(),
                ((Number) json.get("flModuleY")).doubleValue()),
            new Translation2d(
                ((Number) json.get("frModuleX")).doubleValue(),
                ((Number) json.get("frModuleY")).doubleValue()),
            new Translation2d(
                ((Number) json.get("blModuleX")).doubleValue(),
                ((Number) json.get("blModuleY")).doubleValue()),
            new Translation2d(
                ((Number) json.get("brModuleX")).doubleValue(),
                ((Number) json.get("brModuleY")).doubleValue())
          };

      return new RobotConfig(massKG, MOI, moduleConfig, moduleOffsets);
    } else {
      double trackwidth = ((Number) json.get("robotTrackwidth")).doubleValue();

      return new RobotConfig(massKG, MOI, moduleConfig, trackwidth);
    }
  }
}
