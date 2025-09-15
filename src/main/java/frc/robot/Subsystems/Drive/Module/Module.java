package frc.robot.Subsystems.Drive.Module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * creates an instance of a module that can run states and report odometry information call 4 times
 * for all 4 modules 0 -> front left 1-> front right 2 -> back left 3 -> back right
 */
public class Module {
  private final ModuleIO
      io; // the interface used by the module. Will be defined ModuleIOSpark if real or ModuleIOSim
  // if sim in robot
  private final ModuleIOInputsAutoLogged inputs =
      new ModuleIOInputsAutoLogged(); // the logged inputs of the module
  private final int index; // which module it is. 0-3

  // alerts for motor disconnection
  private final Alert driveDCAlert;
  private final Alert turnDCAlert;

  // array of our module positions
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  /**
   * Constructor for the module
   *
   * @param io instance of ModuleIO or classes implementing ModuleIO
   * @param index which module is being created 0 -> front left 1-> front right 2 -> back left 3 ->
   *     back right
   */
  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
    driveDCAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnDCAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
  }
  /**
   * will be called periodically from drive, but won't actually call itself periodically
   * advantagekits logging as well as updating information is NOT thread safe, so call using locks
   * or syncronized
   */
  public void periodic() {
    // update autologged inputs
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    Logger.recordOutput(
        "Drive/Module" + Integer.toString(index) + "testRadians", inputs.turnPosition.getRadians());
    // create a new array of swervemodule positions for the amount of timestamps we have
    int sampleCount = inputs.odometryTimestamps.length;
    odometryPositions = new SwerveModulePosition[sampleCount];

    // fill the odometryPositions array with the module information at givern sample
    for (int i = 0; i < sampleCount; i++) {
      double posMeters =
          inputs.odometryDrivePositionsRad[i]
              * Constants.DriveConstants
                  .wheelRadiusMeters; // calculate how far the drive motor has driven
      Rotation2d angle = inputs.odometryTurnPositions[i]; // find the turn angle
      odometryPositions[i] = new SwerveModulePosition(posMeters, angle); // add to the array
    }

    // update alerts
    driveDCAlert.set(!inputs.driveConnected);
    turnDCAlert.set(!inputs.turnConnected);
  }

  /**
   * sets the module to a desired speed and direction
   *
   * @param state the speed(meters/sec) and angle with which to set the swerve module
   */
  public void runSwerveState(SwerveModuleState state) {
    state.optimize(getAngle()); // reverses the direction so the turn never goes the long way around
    state.cosineScale(inputs.turnPosition); // smooths out the direction change

    // set the motors
    io.setDriveVelo(state.speedMetersPerSecond / Constants.DriveConstants.wheelRadiusMeters);
    io.setTurnPos(state.angle);
  }

  /**
   * runs a motor voltage for characterization
   *
   * @param output volts
   */
  public void runCharacterization(double output) {
    // io.setTurnOpenLoop(output);
    io.setTurnPos(Rotation2d.fromDegrees(0));
    // io.setDriveOpenLoop(output);
  }

  /**
   * runs a motor voltage
   *
   * @param output volts
   */
  public void runOpenLoopTest(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnOpenLoop(output);
  }

  // halts movement of motors
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /**
   * @return the angle of the turn motor in radians
   */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /**
   * @return the meters the drive motor has gone
   */
  public double getPosMeters() {
    return inputs.drivePositionRad * Constants.DriveConstants.wheelRadiusMeters;
  }

  /**
   * @return the velocity of the drive motor in meters per second
   */
  public double getVelo() {
    return inputs.driveVelocityRadPerSec * Constants.DriveConstants.wheelRadiusMeters;
  }

  /**
   * @return the position of the module in meters and radians
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPosMeters(), getAngle());
  }

  /**
   * @return the state of the module in meters/sec and radians
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelo(), getAngle());
  }

  /**
   * @return the array of odometry positions
   */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /**
   * @return the array of odometry timestamps
   */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /**
   * @return the drive motor information for characterizing wheel radius
   */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /**
   * @return the drive motor velo for characterizing feed forward
   */
  public double getFFCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }
}
