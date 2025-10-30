package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.IntStream;

/**
 * This class exists as a universal reference for the bot's positional information, so subsystems
 * that need it do not have to require drive This class also exists so that odmetry classes can have
 * constraints set globally, without needing the class to be unnesesarily called
 */
public class Robotstate {

  /**
   * implementation of singleton, in order to ensure that the Robotstate class is universal and can
   * be called without needing to be defined
   */
  private static Robotstate instance;
  // synchronized because of the possibility that multiple classes call at once, thereby bypassing
  // the check and creating multiple instances
  public static synchronized Robotstate getInstance() {
    if (instance == null) {
      instance = new Robotstate();
    }
    return instance;
  }

  private Pose2d pose; // pose of the bot
  private ChassisSpeeds chassisSpeeds; // chassis speeds of bot in meters/sec
  private double gyroYawVelo; // the gyroscope velocity in radians per sec
  private int[] allowedTagPoses = IntStream.range(1, 23).toArray();

  private List<RobotstatePoseListener> robotstatePoseListeners = new ArrayList<>();

  public void addListener(RobotstatePoseListener listener) {
    robotstatePoseListeners.add(listener);
  }

  /** will be called periodically in drive to update Robotstate's information */
  public synchronized void updateBotPoseAndSpeeds(Pose2d pose, ChassisSpeeds chassisSpeeds) {
    this.pose = pose;
    this.chassisSpeeds = chassisSpeeds;
  }

  /** will be called periodically in drive to update Robotstate's information */
  public synchronized void updateRawGyroVelo(double gyroYawVelo) {
    this.gyroYawVelo = gyroYawVelo;
  }

  /** use this so Robotstate is also reset anytime drive pose is reset */
  public synchronized void setPose(Pose2d pose) {
    this.pose = pose;
  }

  /** call in order to reset the vision subsystem to doing global pose estimation */
  public synchronized void resetAllowedTagPoses() {
    allowedTagPoses = IntStream.range(1, 23).toArray();
  }

  /**
   * call in order to set the vision subsystem to focus on a specific tag for something like
   * autoalign, not a perfect solution, but should reduce extra ambiguity added from other tags see
   * https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2025-build-thread/477314/85
   */
  public synchronized void setAllowedTagPoses(int... allowedTags) {
    allowedTagPoses = allowedTags;
  }

  /** synchronized thread safe getters */
  public synchronized double getAllowedTagPosesLength() {
    return allowedTagPoses.length;
  }

  public synchronized int[] getAllowedTagPoses() {
    return allowedTagPoses;
  }

  public synchronized boolean getIfAllowedTagsSpecified() {
    return getAllowedTagPosesLength() != 22;
  }

  public synchronized Pose2d getPose() {
    return pose;
  }

  public synchronized Rotation2d getRotation() {
    return pose.getRotation();
  }

  public synchronized ChassisSpeeds getSpeeds() {
    return chassisSpeeds;
  }

  public synchronized double getGyroVeloRadPerSec() {
    return gyroYawVelo;
  }

  public void informAllPoseListeners(Pose2d pose) {
    for (RobotstatePoseListener listener : robotstatePoseListeners) {
      listener.accept(pose);
    }
  }

  @FunctionalInterface
  public interface RobotstatePoseListener {
    // could be given more parameters later but for now the only one we care about is the pose
    void accept(Pose2d pose);
  }
}
