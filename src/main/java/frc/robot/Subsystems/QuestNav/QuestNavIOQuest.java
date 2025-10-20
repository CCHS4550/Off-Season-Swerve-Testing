package frc.robot.Subsystems.QuestNav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.ArrayList;
import java.util.List;

public class QuestNavIOQuest implements QuestNavIO {
  protected final QuestNav quest = new QuestNav();
  protected final Transform3d robotToQuest;
  private boolean initialPoseSet = false;

  /**
   * Creates a new QuestIOQuestNav
   *
   * @param robotToQuest The 3D position of the quest relative to the robot.
   */
  public QuestNavIOQuest(Transform3d robotToQuest) {
    this.robotToQuest = robotToQuest;
  }

  @Override
  public void updateInputs(QuestNavIOInputs inputs) {
    inputs.QuestNavConnected = quest.isConnected();
    inputs.QuestNavTracking = quest.isTracking();

    if (!quest.isTracking()) {
      initialPoseSet = false;
    }
    inputs.hasEstablishedSetPose = initialPoseSet;

    // looping over the array of unread poseframs this way should allow us to implement further
    // filtering in the future
    List<PoseFrame> poseFrames = new ArrayList<>();
    for (var i : quest.getAllUnreadPoseFrames()) {
      poseFrames.add(i);
    }
    inputs.unreadPoseFrames = poseFrames.toArray(new PoseFrame[0]);

    inputs.appTimeStamp = quest.getAppTimestamp().orElse(0.0);

    inputs.latencyMS = quest.getLatency();

    inputs.batteryPercent = quest.getBatteryPercent().orElse(0);

    inputs.frameCount = quest.getFrameCount().orElse(0);

    inputs.trackingLostCounter = quest.getTrackingLostCounter().orElse(0);
  }

  public void setPose(Pose2d pose) {
    initialPoseSet = true;

    Pose3d tempPose = new Pose3d(pose);
    tempPose.transformBy(robotToQuest);

    pose = tempPose.toPose2d();

    quest.setPose(pose);
  }
}
