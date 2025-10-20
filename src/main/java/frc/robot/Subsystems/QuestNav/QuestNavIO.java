package frc.robot.Subsystems.QuestNav;

import gg.questnav.questnav.PoseFrame;
import org.littletonrobotics.junction.AutoLog;

// creates a questnav interface for hardware or sim to fully define
public interface QuestNavIO {

  // automatically generates a class using clonable that logs the following variables with advantage
  // scope
  @AutoLog
  public static class QuestNavIOInputs {
    public boolean QuestNavConnected = false;
    public boolean QuestNavTracking = false;
    public boolean hasEstablishedSetPose = false;

    public PoseFrame[] unreadPoseFrames = new PoseFrame[0];

    // NOT the same as data timestamps and should not be used to timestamp information
    public double appTimeStamp = 0.0; // returns optional

    public double latencyMS = 0;
    public int batteryPercent = 0; // returns optional
    public int frameCount = 0; // returns optional
    public int trackingLostCounter = 0; // returns optional
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(QuestNavIOInputs inputs) {}
}
