package frc.robot.Subsystems.Drive.Module;

import static frc.robot.Util.SparkUtil.ifOK;
import static frc.robot.Util.SparkUtil.ifOk;
import static frc.robot.Util.SparkUtil.makeItWork;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;
import frc.robot.Subsystems.Drive.SparkOdometryThread;
import frc.robot.Util.SparkUtil;
import java.util.Queue;
import java.util.function.DoubleSupplier;

// This classes writing is fucked because the absolute encoder isnt part of the spark/ just talk to
// electrical about it

// TODO: make the offset math neater

// define a class that uses the interface ModuleIO
// used to initiate the hardware used on the row and define the interface methods
// call 4 times for all 4 modules
public class ModuleIOSpark implements ModuleIO {

  // the absolute encoder will be off from the true wheel angle, we can use this measurement to
  // account for that
  private final Rotation2d rotationOffset;

  // the motors and encoders of the module
  // declared as sparkbases, which means it could be either MAX or FLEX, we only use maxes
  private final SparkBase driveSpark;
  private final SparkMax turnSpark;
  private final RelativeEncoder driveEncoder;
  // private final AbsoluteEncoder turnEncoder;

  private final AnalogEncoder absoluteEncoder;
  private final AnalogInput absoluteAnalogInput;

  // closed loop control for both motors
  private final SparkClosedLoopController driveController;
  //private final SparkClosedLoopController turnController;

  private final PIDController turnPID =
      new PIDController(Constants.DriveConstants.turnKp, 0.0, Constants.DriveConstants.turnKd);

  // odometry information for the module
  private final Queue<Double> timeQueue;
  private final Queue<Double> drivePosQueue;
  private final Queue<Double> turnPosQueue;

  // checks if a module is disconnected
  private final Debouncer driveDebouncer = new Debouncer(0.5);
  private final Debouncer turnDebouncer = new Debouncer(0.5);

  /**
   * Contructor for the module
   *
   * @param module the module that is being created 0 -> front left 1 -> front right 2 -> back left
   *     3 -> back right
   */
  public ModuleIOSpark(int module) {

    // use switch statement to set offset for currect module
    rotationOffset =
        switch (module) {
          case 0 -> Constants.DriveConstants.frontLeftOffset;
          case 1 -> Constants.DriveConstants.frontRightOffset;
          case 2 -> Constants.DriveConstants.backLeftOffset;
          case 3 -> Constants.DriveConstants.backRightOffset;
          default -> new Rotation2d();
        };
    // use switch statement to declare turn spark with right can ID
    turnSpark =
        new SparkMax(
            switch (module) {
              case 0 -> Constants.DriveConstants.frontLeftTurnCanId;
              case 1 -> Constants.DriveConstants.frontRightTurnCanId;
              case 2 -> Constants.DriveConstants.backLeftTurnCanId;
              case 3 -> Constants.DriveConstants.backRightTurnCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    absoluteAnalogInput =
        new AnalogInput(
            switch (module) {
              case 0 -> Constants.DriveConstants.frontLeftTurnEncoder;
              case 1 -> Constants.DriveConstants.frontRightTurnEncoder;
              case 2 -> Constants.DriveConstants.backLeftTurnEncoder;
              case 3 -> Constants.DriveConstants.backRightTurnEncoder;
              default -> 0;
            });
    absoluteEncoder = new AnalogEncoder(absoluteAnalogInput);
    // use switch statement to declare drive spark with right can ID
    driveSpark =
        new SparkMax(
            switch (module) {
              case 0 -> Constants.DriveConstants.frontLeftDriveCanId;
              case 1 -> Constants.DriveConstants.frontRightDriveCanId;
              case 2 -> Constants.DriveConstants.backLeftDriveCanId;
              case 3 -> Constants.DriveConstants.backRightDriveCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    // declare encoders for both motors
    driveEncoder = driveSpark.getEncoder(); // don't need an absolute encoder for drive
    //turnEncoder = turnSpark.getAbsoluteEncoder();

    // declare closed loop control for both motors
    driveController = driveSpark.getClosedLoopController();
    //turnController = turnSpark.getClosedLoopController();

    turnPID.enableContinuousInput(0, Math.PI);

    // turn motor config
    var turnConfig = new SparkMaxConfig();

    // use switch statement to set correct inverted value
    switch (module) {
      case 0 -> turnConfig.inverted(Constants.DriveConstants.frontLeftTurnInverted);
      case 1 -> turnConfig.inverted(Constants.DriveConstants.frontRightTurnInverted);
      case 2 -> turnConfig.inverted(Constants.DriveConstants.backLeftTurnInverted);
      case 3 -> turnConfig.inverted(Constants.DriveConstants.backRightTurnInverted);
      default -> turnConfig.inverted(false);
    }

    /**
     * idleMode is Brake, stay at position when stopped set the smart current limit to avoid going
     * over what the motor can handle voltage compensation = 12 because working with 12v car battery
     */
    turnConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.DriveConstants.turnMotorCurrentLimit)
        .voltageCompensation(12);

    /**
     * configures the turn encoder position factor converts rotations to radians while accounting
     * for any gearing velocity factor converts rotations/min to radians/sec while accounting for
     * any gearing
     *
     * <p>this is now automatically applied anytime we request motor information
     *
     * <p>average depth is the bit size of the sampling depth, must be a power of 2
     */
    // turnConfig
    //     .absoluteEncoder
    //     .positionConversionFactor(Constants.DriveConstants.turnEncoderPositionFactor)
    //     .velocityConversionFactor(Constants.DriveConstants.turnEncoderVelocityFactor)
    //     .setSparkMaxDataPortConfig()
    //     .averageDepth(2);

    /**
     * each sparkmax supports up to 4 slots for a preconfigured pid that it can then call using
     * SparkClosedLoopController defaults to slot 0 if not specified
     *
     * <p>feedbackSensor sets our sensor to the absolute encoder position wrapping makes it loop
     * from the given range which we set to 0 and 2pi
     *
     * <p>then we set the pid configs ff = 0 because they do not take into account ks and their calc
     * isnt amazing instead we will implement ff as an arbff to be added later
     */
    // turnConfig
    //     .closedLoop
    //     .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
    //     .positionWrappingEnabled(true)
    //     .positionWrappingInputRange(
    //         Constants.DriveConstants.turnPIDMinInput, Constants.DriveConstants.turnPIDMaxInput)
    //     .pidf(Constants.DriveConstants.turnKp, 0.0, Constants.DriveConstants.turnKd, 0.0);

    /**
     * configure how often the turnmotor receives/uses signals
     *
     * <p>we set the encoder to always give output we set the encoder speed to the a calculation
     * converting odometry frequency to ms
     *
     * <p>set the velocity to always give output
     *
     * <p>set every other periodic function in the motor to 20ms the standard
     */
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(
            (int) (1000.0 / Constants.DriveConstants.odometryFrequency))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    /** configures the turn motor, retrying if faulting */
    SparkUtil.makeItWork(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // drive motor config
    var driveConfig = new SparkMaxConfig();

    /** use switch statement to set correct inverted value */
    switch (module) {
      case 0 -> driveConfig.inverted(Constants.DriveConstants.frontLeftDriveInverted);
      case 1 -> driveConfig.inverted(Constants.DriveConstants.frontRightDriveInverted);
      case 2 -> driveConfig.inverted(Constants.DriveConstants.backLeftDriveInverted);
      case 3 -> driveConfig.inverted(Constants.DriveConstants.backRightDriveInverted);
      default -> driveConfig.inverted(false);
    }
    /**
     * idleMode is Brake, means that when no voltage is applied stay at position set the smart
     * current limit to avoid going over what the motor can handle voltage compensation = 12 because
     * working with 12v car battery
     */
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.DriveConstants.driveMotorCurrentLimit)
        .voltageCompensation(12.0);

    /**
     * configures the drive encoder remember that because drive is just the wheel spinning in
     * circles, the encoder position reseting to 0 each restart is not an issue, so not absolute
     * encoder required position factor converts rotations to radians while accounting for any
     * gearing velocity factor converts rotations/min to radians/sec while accounting for any
     * gearing
     *
     * <p>this is now automatically applied anytime we request motor information
     *
     * <p>average depth is the bit size of the sampling depth, must be a power of 2
     */
    driveConfig
        .encoder
        .positionConversionFactor(Constants.DriveConstants.driveEncoderPositionFactor)
        .velocityConversionFactor(Constants.DriveConstants.driveEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    /**
     * each sparkmax supports up to 4 slots for a preconfigured pid that it can then call using
     * SparkClosedLoopController defaults to slot 0 if not specified
     *
     * <p>feedbackSensor sets our sensor to the absolute encoder no pos wrapping because we want the
     * total num of rotations for odometry
     *
     * <p>then we set the pid configs ff = 0 because they do not take into account ks and their calc
     * isnt amazing instead we will implement ff as an arbff to be added later
     */
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(Constants.DriveConstants.driveKp, 0.0, Constants.DriveConstants.driveKd, 0.0);

    /**
     * configure how often the drivemotor receives/uses signals
     *
     * <p>we set the encoder to always give output we set the encoder speed to the a calculation
     * converting odometry frequency to ms
     *
     * <p>set the velocity to always give output
     *
     * <p>set every other periodic function in the motor to 20ms the standard
     */
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / Constants.DriveConstants.odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    /** configures the turn motor, retrying if faulting */
    makeItWork(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    /** reset drive encoder, retrying if faulting */
    makeItWork(
        driveSpark,
        5,
        () -> driveEncoder.setPosition(0.0)); // reset the sparkmax encoder b/c its not absolute

    // make the queues that are filled by the odometry thread
    // odometry thread will periodically fill these queues
    timeQueue = SparkOdometryThread.getInstance().makeTimeQueue();
    drivePosQueue =
        SparkOdometryThread.getInstance()
            .registerSparkSignal(driveSpark, driveEncoder::getPosition);
    turnPosQueue =
        SparkOdometryThread.getInstance()
            .registerSparkSignal(
                turnSpark,
                () ->
                    Rotation2d.fromRotations(absoluteEncoder.get())
                        .minus(rotationOffset)
                        .plus(Rotation2d.kPi)
                        .getRadians());
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // update drive moter values, only accepting if no sticky fault present
    SparkUtil.stickyFault = false;
    ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
    ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
    ifOK(
        driveSpark,
        new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
        (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveDebouncer.calculate(!SparkUtil.stickyFault);

    // update turn motor values, only accepting if no sticky fault present
    SparkUtil.stickyFault = false;
    inputs.turnPosition =
        Rotation2d.fromRotations(absoluteEncoder.get()).minus(rotationOffset).plus(Rotation2d.kPi);
    inputs.turnVelocityRadPerSec = Rotation2d.fromRotations(turnSpark.getEncoder().getVelocity()).getRadians();
    ifOK(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnDebouncer.calculate(!SparkUtil.stickyFault);
    inputs.turnEncoderVolts = getAbsoluteEncoderVolts();

    getAbsoluteEncoderVolts();

    // add the odometry information from the queue to the autologged array, then clear the queue
    inputs.odometryTimestamps = timeQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePosQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPosQueue.stream()
            .map((Double value) -> new Rotation2d(value).minus(rotationOffset))
            .toArray(Rotation2d[]::new);
    timeQueue.clear();
    drivePosQueue.clear();
    turnPosQueue.clear();
  }

  /**
   * sets drive motor to specified open loop value
   *
   * @param output the volts to set
   */
  @Override
  public void setDriveOpenLoop(double output) {
    driveSpark.setVoltage(output);
  }

  /**
   * sets turn motor to specified open loop value
   *
   * @param output the volts to set
   */
  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  /**
   * set drive motor to a desired velocity while calculating a feedforward value to feed to the
   * spark controller
   *
   * @param velo velocity in radians per second
   */
  @Override
  public void setDriveVelo(double velo) {
    double ffvolts =
        Constants.DriveConstants.driveKs * Math.signum(velo)
            + Constants.DriveConstants.driveKv * velo;
    driveController.setReference(
        velo, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffvolts, ArbFFUnits.kVoltage);
  }

  /**
   * set turn motor to a desire angle, while wrapping the setpoint to prevent invalid inputs and
   * account for the motor offset no feedforward because there is no velocity goal to achieve
   *
   * @param rotation desire module angle in radians
   */
  @Override
  public void setTurnPos(Rotation2d rotation) {
    double setPoint =
        MathUtil.inputModulus(
            rotation.plus(rotationOffset).getRadians(),
            Constants.DriveConstants.turnPIDMinInput,
            Constants.DriveConstants.turnPIDMaxInput);
    double volts =
        turnPID.calculate(
            (Rotation2d.fromRotations(absoluteEncoder.get()).plus(Rotation2d.kPi)).getRadians(),
            setPoint);
    System.out.println(volts);
    setTurnOpenLoop(volts);
  }

  public double getAbsoluteEncoderVolts() {
    return absoluteAnalogInput.getVoltage();
  }

  public double getRawRadians() {
    return Rotation2d.fromRotations(absoluteEncoder.get()).getRadians();
  }
}
