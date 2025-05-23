package com.pathplanner.lib.util;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.lib.ftclib.opmode.Robot;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Filesystem;

/** Utility class for sending data to the PathPlanner app via NT4 */
public class PPLibTelemetry {
  private static boolean compMode = false;

  private static final DoubleArrayPublisher velPub =
      NetworkTableInstance.getDefault().getDoubleArrayTopic("/PathPlanner/vel").publish();
  private static final StructPublisher<Pose2d> posePub =
      NetworkTableInstance.getDefault()
          .getStructTopic("/PathPlanner/currentPose", Pose2d.struct)
          .publish();
  private static final StructArrayPublisher<Pose2d> pathPub =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("/PathPlanner/activePath", Pose2d.struct)
          .publish();
  private static final StructPublisher<Pose2d> targetPosePub =
      NetworkTableInstance.getDefault()
          .getStructTopic("/PathPlanner/targetPose", Pose2d.struct)
          .publish();

  private static final Map<String, List<PathPlannerPath>> hotReloadPaths = new HashMap<>();
  private static final Map<String, List<PathPlannerAuto>> hotReloadAutos = new HashMap<>();
  private static NetworkTableListener hotReloadPathListener = null;
  private static NetworkTableListener hotReloadAutoListener = null;

  /** Enable competition mode. This will disable hot reload. */
  public static void enableCompetitionMode() {
    compMode = true;
  }

  /**
   * Set the path following actual/target velocities
   *
   * @param actualVel Actual robot velocity in m/s
   * @param commandedVel Target robot velocity in m/s
   * @param actualAngVel Actual angular velocity in rad/s
   * @param commandedAngVel Target angular velocity in rad/s
   */
  public static void setVelocities(
      double actualVel, double commandedVel, double actualAngVel, double commandedAngVel) {
    if (!compMode) {
      velPub.set(new double[] {actualVel, commandedVel, actualAngVel, commandedAngVel});
    }
  }

  /**
   * Set the current robot pose
   *
   * @param pose Current robot pose
   */
  public static void setCurrentPose(Pose2d pose) {
    if (!compMode) {
      posePub.set(pose);
    }
  }

  /**
   * Set the current path being followed
   *
   * @param path The current path
   */
  public static void setCurrentPath(PathPlannerPath path) {
    if (!compMode) {
      // Use poses for simplicity
      pathPub.set(path.getPathPoses().toArray(new Pose2d[0]));
    }
  }

  /**
   * Set the target robot pose
   *
   * @param targetPose Target robot pose
   */
  public static void setTargetPose(Pose2d targetPose) {
    if (!compMode) {
      targetPosePub.set(targetPose);
    }
  }

  /**
   * Register a path for hot reload. This is used internally.
   *
   * @param pathName Name of the path
   * @param path Reference to the path
   */
  public static void registerHotReloadPath(String pathName, PathPlannerPath path) {
    if (!compMode) {
      ensureHotReloadListenersInitialized();
      if (!hotReloadPaths.containsKey(pathName)) {
        hotReloadPaths.put(pathName, new ArrayList<>());
      }

      hotReloadPaths.get(pathName).add(path);
    }
  }

  /**
   * Register an auto for hot reload. This is used internally.
   *
   * @param autoName Name of the auto
   * @param auto Reference to the auto
   */
  public static void registerHotReloadAuto(String autoName, PathPlannerAuto auto) {
    if (!compMode) {
      ensureHotReloadListenersInitialized();
      if (!hotReloadAutos.containsKey(autoName)) {
        hotReloadAutos.put(autoName, new ArrayList<>());
      }

      hotReloadAutos.get(autoName).add(auto);
    }
  }

  private static void ensureHotReloadListenersInitialized() {
    if (hotReloadPathListener == null) {
      hotReloadPathListener =
          NetworkTableListener.createListener(
              NetworkTableInstance.getDefault()
                  .getStringTopic("/PathPlanner/HotReload/hotReloadPath"),
              EnumSet.of(NetworkTableEvent.Kind.kValueRemote),
              PPLibTelemetry::handlePathHotReloadEvent);
    }
    if (hotReloadAutoListener == null) {
      hotReloadAutoListener =
          NetworkTableListener.createListener(
              NetworkTableInstance.getDefault()
                  .getStringTopic("/PathPlanner/HotReload/hotReloadAuto"),
              EnumSet.of(NetworkTableEvent.Kind.kValueRemote),
              PPLibTelemetry::handleAutoHotReloadEvent);
    }
  }

  private static void handlePathHotReloadEvent(NetworkTableEvent event) {
    if (!compMode) {
      if (!Robot.isDisabled()) {
        RobotLog.w("Ignoring path hot reload, robot is enabled", false);
        return;
      }

      try {
        String jsonStr = event.valueData.value.getString();

        JSONObject json = (JSONObject) new JSONParser().parse(jsonStr);
        String name = (String) json.get("name");
        JSONObject pathJson = (JSONObject) json.get("path");

        if (hotReloadPaths.containsKey(name)) {
          for (PathPlannerPath path : hotReloadPaths.get(name)) {
            path.hotReload(pathJson);
          }
        }

        File pathFile =
            new File(Filesystem.getDeployDirectory(), "pathplanner/paths/" + name + ".path");

        try (FileWriter writer = new FileWriter(pathFile)) {
          writer.write(pathJson.toJSONString());
          writer.flush();
        } catch (IOException e) {
          RobotLog.w(
              "Failed to save updated path file contents, please re-deploy code", false);
        }

      } catch (Exception e) {
        // Ignore
      }
    }
  }

  private static void handleAutoHotReloadEvent(NetworkTableEvent event) {
    if (!compMode) {
//      if (DriverStation.isEnabled()) {
//        DriverStation.reportWarning("Ignoring auto hot reload, robot is enabled", false);
//        return;
//      }

      try {
        String jsonStr = event.valueData.value.getString();

        JSONObject json = (JSONObject) new JSONParser().parse(jsonStr);
        String name = (String) json.get("name");
        JSONObject autoJson = (JSONObject) json.get("auto");

        if (hotReloadAutos.containsKey(name)) {
          for (PathPlannerAuto auto : hotReloadAutos.get(name)) {
            auto.hotReload(autoJson);
          }
        }


        File pathFile =
            new File(Filesystem.getDeployDirectory(), "pathplanner/autos/" + name + ".auto");

        try (FileWriter writer = new FileWriter(pathFile)) {
          writer.write(autoJson.toJSONString());
          writer.flush();
        } catch (IOException e) {
//          DriverStation.reportWarning(
//              "Failed to save updated auto file contents, please re-deploy code", false);
        }

      } catch (Exception e) {
        // Ignore
      }
    }
  }
}
