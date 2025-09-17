import frc.robot.Subsystems.Vision.VisionIO;


import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
/**
 * Interface with the QuestNav on VR headset for pose estimation. See
 * https://www.chiefdelphi.com/t/questnav-the-best-robot-pose-tracking-system-in-frc/
 */

/**  Helpful resource for understanding this stuff
 * https://docs.wpilib.org/en/stable/docs/software/networktables/index.html
 */


public class VisionIOQuestNav implements VisionIO{

    // quick networktables term guide
    // networktables acts very similar to advantagescope, in which it is a way to publish data, but it is more used for data communication b/w the computer and devices on bot
    // topic = channel of data, w/ fixed data type
    // publisher = sends regular/periodic/timestamped values to the topic (kind of like a setter)
    // subscriber = recieves data values from the topic (kind of like a getter)
    // entry = combines the publisher and subscriber role into one
    // property = metadata about a topic

    // in essence, the ideas behind it are very similar to advantagekit and logging

    /* Taken from Chief Delphi, 2383 Ninjineers, 1732 Hilltopper Robotics */
    // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
    NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault(); // basically gets a "copy" of network tables we can access
    NetworkTable nt4Table = nt4Instance.getTable("questnav");
    private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
    private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();

    // Subscribe to the Network Tables questnav data topics
    private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
    private FloatArraySubscriber questPosition =
        nt4Table.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
    private FloatArraySubscriber questQuaternion =
        nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});
    private FloatArraySubscriber questEulerAngles =
        nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});
    private DoubleSubscriber questBatteryPercent =
        nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);
    /* End of stolen stuff */


    @Override
    public static void periodic (VisionIOInputs io){
        //Logging stuff

    }
    
    
}
