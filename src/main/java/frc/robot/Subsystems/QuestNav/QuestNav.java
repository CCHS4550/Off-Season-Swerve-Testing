package frc.robot.Subsystems.QuestNav;

import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Vision.Vision;
import gg.questnav.questnav.PoseFrame;

public class QuestNav extends SubsystemBase implements Vision.VisionConsumer{
    private final questConsumer consumer;

    private final QuestNavIO io;
    private final QuestNavIOInputsAutoLogged inputs = new QuestNavIOInputsAutoLogged();

    private final Alert disconnectedAlert;
    private final Alert noTrackingAlert;


    // because the questnav is such a robust odometry system, these standard deviations realistically should not need to change.
    private final Matrix<N3, N1> QUESTNAV_STD_DEVS =
    VecBuilder.fill(
        0.02, // Trust down to 2cm in X direction
        0.02, // Trust down to 2cm in Y direction
        0.035 // Trust down to 2 degrees rotational
    );

    public QuestNav(questConsumer consumer, QuestNavIO io){
        this.consumer = consumer;
        this.io = io;

        disconnectedAlert = new Alert("The quest is disconnected", AlertType.kWarning);
        noTrackingAlert = new Alert("The quest is not tracking", AlertType.kWarning);
    }

    @Override
    public void periodic(){
        io.commandPeriodic();
        synchronized(inputs){
            io.updateInputs(inputs);
            Logger.processInputs("Questnav", inputs);

            List<Pose2d> allPoseFrames = new LinkedList<>();
            List<Pose2d> rejectedPoseFrames = new LinkedList<>();
            List<Pose2d> acceptedPoseFrames = new LinkedList<>();

            for(PoseFrame givenPoseFrame : inputs.unreadPoseFrames){
                boolean rejectFrame = !inputs.QuestNavTracking || !inputs.hasEstablishedSetPose;

                allPoseFrames.add(givenPoseFrame.questPose());
                if(rejectFrame){
                    rejectedPoseFrames.add(givenPoseFrame.questPose());
                }
                else{
                    acceptedPoseFrames.add(givenPoseFrame.questPose());

                    Pose3d transformedPose = new Pose3d(givenPoseFrame.questPose()).transformBy(inputs.robotToQuest.inverse());

                    consumer.accept(transformedPose.toPose2d(), givenPoseFrame.dataTimestamp(), QUESTNAV_STD_DEVS);
                }
            }

            Logger.recordOutput("Questnav", allPoseFrames.toArray(new Pose2d[allPoseFrames.size()]));
            Logger.recordOutput("Questnav", rejectedPoseFrames.toArray(new Pose2d[rejectedPoseFrames.size()]));
            Logger.recordOutput("Questnav", acceptedPoseFrames.toArray(new Pose2d[acceptedPoseFrames.size()]));

            disconnectedAlert.set(!inputs.QuestNavConnected);
            noTrackingAlert.set(!inputs.QuestNavTracking);
        }
    }

    public void setPose(Pose2d pose){
        io.setPose(pose);
    }
    
    @FunctionalInterface
    public interface questConsumer{
        void accept(Pose2d visionRobotPoseMeters,
        double timestampSeconds, Matrix<N3, N1> questMeasurementStdDevs);
    }

    @Override
    public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        Pose3d transformedPose = new Pose3d(visionRobotPoseMeters).transformBy(inputs.robotToQuest);
        
        setPose(transformedPose.toPose2d());
    }
}
