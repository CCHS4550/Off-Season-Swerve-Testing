package frc.robot.Subsystems.Drive.Module;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/** Physics sim implementation of module IO. */
public class ModuleIOSim implements ModuleIO {
  // the overall simulator for module physics
  private final SwerveModuleSimulation moduleSimulation;

  // the physics sim for individual motors
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController turnMotor;

  // closed loop control
  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private final PIDController driveController = new PIDController(driveSimP, 0, driveSimD);
  private final PIDController turnController = new PIDController(turnSimP, 0, turnSimD);

  // feed forward control
  private double driveFFVolts = 0.0;

  // calculate inputs off applied volts
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  // constructor for the module sim
  public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;
    this.driveMotor =
        moduleSimulation
            .useGenericMotorControllerForDrive()
            .withCurrentLimit(Amps.of(driveMotorCurrentLimit));
    this.turnMotor =
        moduleSimulation
            .useGenericControllerForSteer()
            .withCurrentLimit(Amps.of(turnMotorCurrentLimit));

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts
              + driveController.calculate(
                  moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond));
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts =
          turnController.calculate(moduleSimulation.getSteerAbsoluteFacing().getRadians());
    } else {
      turnController.reset();
    }

    // Update simulation state
    driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
    turnMotor.requestVoltage(Volts.of(turnAppliedVolts));

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocityRadPerSec = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(moduleSimulation.getDriveMotorStatorCurrent().in(Amps));

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnPosition = moduleSimulation.getSteerAbsoluteFacing();
    inputs.turnVelocityRadPerSec =
        moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Amps));

    // Update odometry inputs
    inputs.odometryTimestamps = frc.robot.Util.SparkUtil.getSimulationOdometryTimeStamps();
    inputs.odometryDrivePositionsRad =
        Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();
    inputs.odometryTurnPositions = moduleSimulation.getCachedSteerAbsolutePositions();
  }

  /**
   * sets drive motor to specified open loop value
   *
   * @param output the volts to set
   */
  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  /**
   * sets turn motor to specified open loop value
   *
   * @param output the volts to set
   */
  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  /**
   * set drive motor to a desired velocity while calculating a feedforward value to feed to the
   * simulation
   *
   * <p>simply sets the setpoint for closed loop and the calculation is done in update inputs
   *
   * @param velo velocity in radians per second
   */
  @Override
  public void setDriveVelo(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = driveSimKs * Math.signum(velocityRadPerSec) + driveSimKv * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  /**
   * set turn motor to a desire angle
   *
   * <p>simply sets the setpoint for closed loop and the calculation is done in update inputs
   *
   * @param rotation desire module angle in radians
   */
  @Override
  public void setTurnPos(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
