package frc.robot.Subsystems.Drive.Gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.Subsystems.Drive.SparkOdometryThread;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroPigeon implements GyroIO {

  // initiates a pigeon
  private final Pigeon2 pigeon = new Pigeon2(Constants.DriveConstants.pigeonCanId);

  // creates information lists
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroPigeon() {
    // configure the pigeon
    pigeon.getConfigurator().apply(new Pigeon2Configuration());

    // set its rotation to 0
    pigeon.getConfigurator().setYaw(0.0);

    // set update frquencies
    yaw.setUpdateFrequency(Constants.DriveConstants.odometryFrequency);
    yawVelocity.setUpdateFrequency(50.0);

    // reduce the Bus uitlization by disabling/reducing unused signals
    pigeon.optimizeBusUtilization();

    // make the queues that are filled by the odometry thread
    // odometry thread will periodically fill these queues
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimeQueue();
    var yawClone = yaw.clone(); // Status signals are not thread-safe
    yawPositionQueue =
        SparkOdometryThread.getInstance()
            .registerGenericSignal(() -> yawClone.refresh().getValueAsDouble());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // if connected
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);

    // log position and velocity
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    // add the odometry information from the queue to the autologged array, then clear the queue
    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
