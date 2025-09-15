package frc.robot.Subsystems.Drive.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

// creates an interface for hardware to fully defin
public interface GyroIO {

  // automatically generates a class using clonable that logs the following variables with advantage
  // scope
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = new Rotation2d();
    public double yawVelocityRadPerSec = 0.0;
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(GyroIOInputs inputs) {}
}
