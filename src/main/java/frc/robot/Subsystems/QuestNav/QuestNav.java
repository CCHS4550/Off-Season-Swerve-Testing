package frc.robot.Subsystems.QuestNav;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robotstate;
import frc.robot.Subsystems.Vision.Vision;
import gg.questnav.questnav.PoseFrame;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class QuestNav extends SubsystemBase
    implements Vision.VisionConsumer, Robotstate.RobotstatePoseListener {
  private final QuestConsumer consumer;

  private final QuestNavIO io;
  private final QuestNavIOInputsAutoLogged inputs = new QuestNavIOInputsAutoLogged();

  private final Alert disconnectedAlert;
  private final Alert noTrackingAlert;

  private final Timer timer = new Timer();

  // because the questnav is such a robust odometry system, these standard deviations realistically
  // should not need to change.
  private final Matrix<N3, N1> QUESTNAV_STD_DEVS =
      VecBuilder.fill(
          0.02, // Trust down to 2cm in X direction
          0.02, // Trust down to 2cm in Y direction
          0.035 // Trust down to 2 degrees rotational
          );

  public QuestNav(QuestConsumer consumer, QuestNavIO io) {
    this.consumer = consumer;
    this.io = io;

    disconnectedAlert = new Alert("The quest is disconnected", AlertType.kWarning);
    noTrackingAlert = new Alert("The quest is not tracking", AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.commandPeriodic();
    synchronized (inputs) {
      io.updateInputs(inputs);
      Logger.processInputs("Questnav", inputs);

      List<Pose2d> allPoseFrames = new LinkedList<>();
      List<Pose2d> rejectedPoseFrames = new LinkedList<>();
      List<Pose2d> acceptedPoseFrames = new LinkedList<>();

      for (PoseFrame givenPoseFrame : inputs.unreadPoseFrames) {
        boolean rejectFrame = !inputs.QuestNavTracking || !inputs.hasEstablishedSetPose;

        allPoseFrames.add(givenPoseFrame.questPose());
        if (rejectFrame) {
          rejectedPoseFrames.add(givenPoseFrame.questPose());
        } else {
          acceptedPoseFrames.add(givenPoseFrame.questPose());

          Pose3d transformedPose =
              new Pose3d(givenPoseFrame.questPose()).transformBy(inputs.robotToQuest.inverse());

          consumer.accept(
              transformedPose.toPose2d(), givenPoseFrame.dataTimestamp(), QUESTNAV_STD_DEVS);
        }
      }

      Logger.recordOutput("Questnav", allPoseFrames.toArray(new Pose2d[allPoseFrames.size()]));
      Logger.recordOutput(
          "Questnav", rejectedPoseFrames.toArray(new Pose2d[rejectedPoseFrames.size()]));
      Logger.recordOutput(
          "Questnav", acceptedPoseFrames.toArray(new Pose2d[acceptedPoseFrames.size()]));

      backupOdometrySetter();

      disconnectedAlert.set(!inputs.QuestNavConnected);
      noTrackingAlert.set(!inputs.QuestNavTracking);
    }
  }

  public synchronized void setPose(
      Pose2d
          pose) { // needs to be synchronized b/c vision & the backup odometry could hypothetically
    // try to update the pose at same time
    io.setPose(pose);
  }

  private void backupOdometrySetter() {
    if (!inputs.hasEstablishedSetPose && !timer.isRunning()) {
      timer.reset();
      timer.start();
    }

    if (timer.hasElapsed(5)) {
      setPose(Robotstate.getInstance().getPose());
      timer.stop();
      timer.reset();
    }
  }

  @FunctionalInterface
  public interface QuestConsumer {
    void accept(
        Pose2d questRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> questMeasurementStdDevs);
  }

  @Override
  public void accept(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    Pose3d transformedPose = new Pose3d(visionRobotPoseMeters).transformBy(inputs.robotToQuest);

    setPose(transformedPose.toPose2d());
  }

  @Override
  public void accept(Pose2d pose) {
    Pose3d transformedPose = new Pose3d(pose).transformBy(inputs.robotToQuest);

    setPose(transformedPose.toPose2d());
  }
}
