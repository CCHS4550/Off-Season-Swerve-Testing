package frc.robot.Subsystems.Drive.Gyro;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.util.Units;
import frc.robot.Util.SparkUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;

/** IO implementation for gyro simulation */
public class GyroIOSim implements GyroIO {
  private final GyroSimulation gyroSimulation;

  // create the simulated gyro
  public GyroIOSim(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  // update the autologged inputs with information from the simulated gyro
  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = gyroSimulation.getGyroReading();
    inputs.yawVelocityRadPerSec =
        Units.degreesToRadians(gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond));

    inputs.odometryYawTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
    inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
  }
}
