package frc.robot.Subsystems.Drive.Module;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

// creates an interface for hardware to fully define
public interface ModuleIO {
  // automatically generates a class using clonable that logs the following variables with advantage
  // scope
  @AutoLog
  public static class ModuleIOInputs {
    // tracking variables for the drive motor
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    // tracking variables for the turn motor
    public boolean turnConnected = false;
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;
    public double turnEncoderVolts = 0.0;

    // tracking the odometry queues
    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  public default void setDriveOpenLoop(double output) {}

  /** Run the turn motor at the specified open loop value. */
  public default void setTurnOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelo(double velocityRadPerSec) {}

  /** Run the turn motor to the specified rotation. */
  public default void setTurnPos(Rotation2d rotation) {}
}
