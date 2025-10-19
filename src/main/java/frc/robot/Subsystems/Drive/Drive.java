package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Robotstate;
import frc.robot.Subsystems.Drive.Gyro.GyroIO;
import frc.robot.Subsystems.Drive.Gyro.GyroIOInputsAutoLogged;
import frc.robot.Subsystems.Drive.Module.Module;
import frc.robot.Subsystems.Drive.Module.ModuleIO;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Util.LocalADStarAK;
import java.util.Set;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * this creates a drivetrain that tracks on field positions, and is able to move the entire bot
 * according to input uses a state machine function, so most operations should be able to be called
 * by simply changing the wanted state
 */
public class Drive extends SubsystemBase implements Vision.VisionConsumer {

  // java lock to implement thread safe
  static final Lock odometryLock = new ReentrantLock();

  Transform2d tagTransform =
      new Transform2d(0.0, Units.inchesToMeters(20), Rotation2d.fromDegrees(150));
  Pose2d testPose =
      new Pose2d(
          Constants.VisionConstants.aprilTagLayout
              .getTagPose(20)
              .get()
              .toPose2d()
              .plus(tagTransform)
              .getTranslation(),
          Rotation2d.fromDegrees(60));
  Pose2d testPoseOG = Constants.VisionConstants.aprilTagLayout.getTagPose(20).get().toPose2d();

  // declare gyro
  private final GyroIO
      gyroIO; // the gyro interface used by drive, will be defined as gyroPigeon if real
  private final GyroIOInputsAutoLogged gyroInputs =
      new GyroIOInputsAutoLogged(); // the logged gyro inputs

  private final Module[] modules = new Module[4]; // the 4 modules

  private final SysIdRoutine sysId;

  // configure gyro disconnection alert
  private final Alert gyroDCAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  // used to perform inverse kinematics to convert chassis speeds to individual module states
  private SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(Constants.DriveConstants.moduleTranslations);

  // variable used to track rotation of the robot
  private Rotation2d rawGyroRotation = new Rotation2d();

  // values to use during teleop, these will be periodically set during the default command
  private double xJoystickInput = 0.0;
  private double yJoystickInput = 0.0;
  private double omegaJoystickInput = 0.0;

  // angle for teleop drive but the bot is at a fixed angle
  private Rotation2d joystickDriveAtAngleAngle = Rotation2d.fromRadians(0.0);

  // constraints for drive to point functionality
  private double maxOptionalTurnVeloRadiansPerSec = Double.NaN;
  private double maxVelocityOutputForDriveToPoint = Units.feetToMeters(10.0);

  private APConstraints autoPilotConstraints =
      new APConstraints(
          Constants.DriveConstants.AP_MAXACCEL_METERSPERSECSQUARED,
          Constants.DriveConstants.AP_MAXJERK_METERSPERSECCUBED);
  private APProfile autoPilotProfile =
      new APProfile(autoPilotConstraints)
          .withBeelineRadius(Constants.DriveConstants.AP_BEELINE_RADIUS)
          .withErrorXY(Constants.DriveConstants.AP_XY_ACCEPTED_ERROR)
          .withErrorTheta(Constants.DriveConstants.AP_THETA_ACCEPTED_ERROR);

  private Autopilot autopilot = new Autopilot(autoPilotProfile);

  private APTarget autoPilotTarget = new APTarget(testPose).withVelocity(0.0);

  // values used for pathfinding to pose
  private Pose2d pathOntheFlyPose;
  private PathConstraints pathConstraintsOnTheFly;
  private double maxTransSpeedMpsOnTheFly = 20.0;
  private double maxTransAccelMpssqOnTheFly = 30;
  private double maxRotSpeedRadPerSecOnTheFly = 6;
  private double maxRotAccelRadPerSecSqOnTheFly = 10;
  private double idealEndVeloOntheFly = 0;

  /**
   * Pid Controller for drive at angle.
   *
   * <p>creates a new pid controller with built in trapezoidal motion this is necesary because while
   * the pid controller built into the turn motor can handle turning a singular wheel to an angle, a
   * seperate pid must be called to give the overall angles that the bot is hitting
   */
  ProfiledPIDController angleController =
      new ProfiledPIDController(
          Constants.DriveConstants.ANGLE_KP,
          0.0,
          Constants.DriveConstants.ANGLE_KD,
          new TrapezoidProfile.Constraints(
              Constants.DriveConstants.ANGLE_MAX_VELOCITY,
              Constants.DriveConstants.ANGLE_MAX_ACCELERATION));

  // pid controllers for drive to point, not fully tested so unsure if seperation of auto and teleop
  // is needed, but lower auto values also mean slower more accurate pid
  private final PIDController autoDriveToPointController = new PIDController(0.3, 0, 0.1);
  private final PIDController teleopDriveToPointController = new PIDController(0.69, 0, 0.1);
  private Pose2d driveToPointPose = new Pose2d(); // pose to drive to

  // acceptable margin of error when going to a pose in drive to point
  private static final double goToPoseTranslationError = Units.inchesToMeters(1);
  // acceptable margin of error when going to a pose in path on the fly, larger b/c path on the fly
  // is meant for larger overall translations in this context and is less precise overall
  private static final double pathOntheFlyTranslationError = Units.inchesToMeters(4);

  // potential bad practice
  private BooleanSupplier shouldExecuteCommand = () -> false;
  public final Trigger executeCommand = new Trigger(shouldExecuteCommand);

  private boolean isAtDesiredPose = false;

  // use to exit  drive commands early and make the automate the transition back
  private BooleanSupplier shouldCancelCommandEarly = () -> false;

  // used for drive simulation
  // wont be used unless for sim
  @SuppressWarnings("unused")
  private final Consumer<Pose2d> resetSimulationPoseCallBack;

  public enum WantedState {
    SYS_ID, // COMMAND CONTROL
    AUTO,
    TELEOP_DRIVE,
    TELEOP_DRIVE_AT_ANGLE,
    DRIVE_TO_POINT,
    AUTOPILOT,
    PATH_ON_THE_FLY, // COMMAND CONTROL
    IDLE
  }

  // state the drive train is in
  private enum SystemState {
    SYS_ID, // COMMAND CONTROL
    AUTO,
    TELEOP_DRIVE,
    TELEOP_DRIVE_AT_ANGLE,
    DRIVE_TO_POINT,
    AUTOPILOT,
    PATH_ON_THE_FLY, // COMMAND CONTROL
    IDLE
  }

  // which sys id routine to run
  private enum SysIdtoRun {
    NONE,
    DRIVE_Wheel_Radius_Characterization,
    FEED_FORWARD_Calibration,
    QUASISTATIC_FORWARD,
    QUASISTATIC_REVERSE,
    DYNAMIC_FORWARD,
    DYNAMIC_REVERSE
  }

  // the command that we are running
  private enum CommandtoRun {
    NONE,
    SYS_ID,
    PATH_ON_THE_FLY
  }

  // initialize our states
  private SystemState systemState = SystemState.TELEOP_DRIVE;
  private WantedState wantedState = WantedState.TELEOP_DRIVE;
  private SysIdtoRun sysIdtoRun = SysIdtoRun.NONE;
  private CommandtoRun commandtoRun = CommandtoRun.NONE;

  // array of our previous module positions
  private SwerveModulePosition[] lastModulePos =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  // creates a swervedrive pose estimator, can be used to fuse with vision and does the math needed
  // to update given our bots information
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePos, new Pose2d());

  /**
   * constructor for drive train
   *
   * @param gyroIO instance of gyroIO or classes that implement gyroIO
   * @param flModuleIO instance of moduleIO or classes that implement moduleIO
   * @param frModuleIO instance of moduleIO or classes that implement moduleIO
   * @param blModuleIO instance of moduleIO or classes that implement moduleIO
   * @param brModuleIO instance of moduleIO or classes that implement moduleIO
   */
  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      Consumer<Pose2d> resetSimulationPoseCallBack) {

    // initialize the gyro and modules and sim
    this.gyroIO = gyroIO;
    this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // initiate robot state with null values
    // TODO: maybe do this better
    Robotstate.getInstance().updateBotPoseAndSpeeds(new Pose2d(), new ChassisSpeeds());
    Robotstate.getInstance().updateRawGyroVelo(0.0);

    // Usage reporting
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // begin the odometry thread
    SparkOdometryThread.getInstance().start();

    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(1.6, 0.0, 0.3), new PIDConstants(5.0, 0.0, 0.0)),
        Constants.DriveConstants.ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);

    setPathConstraintsOnTheFly();
    // make sure our angle controller wraps angles properly
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // use our logged AD* algorithm as the pathfinder
    Pathfinding.setPathfinder(new LocalADStarAK());

    /*  behavior of our drive command scheduler is dictated here
     *  the onTrue and onFalse notation means that on any given switch between states, the behavior is performed,
     *  meaning that any given command can only be called once a time even when periodically dealing setting these variables,
     *  as it only schedules on a CHANGE in value.
     *
     *  <p> the onTrue references an enum, meaning that we MUST ensure that the correct command is chosen for that given instance or it will not function properly.
     *  This allows us to make sure all the commands stay working around one central scheduler
     *
     *  <p> note that because triggers are periodically checked by the command scheduler, we should be able to simply set these conditions here and allow the command scheduler to do the rest for us
     */
    executeCommand.onTrue(runCommandControl());
    executeCommand.onFalse(halt());

    // logging
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // create the sysID routine
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    Logger.recordOutput("testing/adjust", testPose);
    Logger.recordOutput("testing/og", testPoseOG);

    setDriveToPointPose(new Pose2d(3, 3, new Rotation2d()));
  }

  @Override
  public void periodic() {
    // lock the thread for thread safe
    odometryLock.lock();

    // update and log gyro inputs
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    // run modules periodic method
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // stop if disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      shouldExecuteCommand = () -> false;
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // odometry calcs
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // get an array of timestamps from modules
    int sampleCount = sampleTimestamps.length; // number of samples to work through

    // loop for all samples of odometry information
    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] modulePositions =
          new SwerveModulePosition[4]; // position of each module
      SwerveModulePosition[] moduleDeltas =
          new SwerveModulePosition[4]; // change in position of each module
      // loop through all modules in a given sample timestamp
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] =
            modules[moduleIndex].getOdometryPositions()[i]; // set the position at given timestamp

        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePos[moduleIndex].distanceMeters,
                modulePositions[moduleIndex]
                    .angle); // find difference in the module position of given timestamp vs
        // previous
        lastModulePos[moduleIndex] =
            modulePositions[moduleIndex]; // set previous timestamp module position
      }

      // update gyro information
      if (gyroInputs.connected) {
        rawGyroRotation = gyroInputs.odometryYawPositions[i]; // update using our real gyro
      } else {
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation =
            rawGyroRotation.plus(new Rotation2d(twist.dtheta)); // update using robot odometry
      }

      // alert of gyro is disconnected
      gyroDCAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

      poseEstimator.updateWithTime(
          sampleTimestamps[i],
          rawGyroRotation,
          modulePositions); // use poseEstimators inbuild function to update with our newest
      // odometry information, coordinated with timestamps
    }

    systemState = handleStateTransition(); // adjust system state according to the wanted state

    // log states
    Logger.recordOutput("Subsystems/Drive/SystemState", systemState);
    Logger.recordOutput("Subsystems/Drive/DesiredState", wantedState);

    // see method comment
    isAtDesiredPose = cancelIfNearAndReturnTrue();

    // turn the states into desired output
    applyStates();

    // for (int i = 0; i < 4; i++) {
    //   modules[i].runCharacterization(10);
    // }

    Robotstate.getInstance().updateBotPoseAndSpeeds(getPose(), getChassisSpeeds());
    Robotstate.getInstance().updateRawGyroVelo(gyroInputs.yawVelocityRadPerSec);

    Logger.recordOutput("Subsystems/Drive/DriveDesiredPoint", driveToPointPose);

    Logger.recordOutput("Subsystems/Drive/ isAtDesiredPose", isAtDesiredPose);
    Logger.recordOutput("Subsystems/Drive/ early cancel", shouldCancelCommandEarly.getAsBoolean());
  }

  /**
   * sets the system state to be the same as the wanted state, but can be set to perform more
   * complex judgements on what state to goto if so desired
   *
   * @return the systemstate that our systemState variable will be set to
   */
  private SystemState handleStateTransition() {

    // if we are not in a command control state, set the early cancel to true
    // and reset the command trigger to false to be ready for a new command to be scheduled
    if (wantedState != WantedState.PATH_ON_THE_FLY && wantedState != WantedState.SYS_ID) {
      setEarlyCommandCancel(true);
      shouldExecuteCommand = () -> false;
    }

    // switch states if the command ought to end early so we dont get stuck in the command control
    // state
    if (shouldCancelCommandEarly.getAsBoolean()
        && (wantedState == WantedState.PATH_ON_THE_FLY || wantedState == WantedState.SYS_ID)) {
      if (DriverStation.isAutonomous()) {
        setWantedState(WantedState.AUTO);
      } else {
        setWantedState(WantedState.TELEOP_DRIVE);
      }
    }

    return switch (wantedState) {
      case SYS_ID -> SystemState.SYS_ID;
      case AUTO -> SystemState.AUTO;
      case TELEOP_DRIVE -> SystemState.TELEOP_DRIVE;
      case TELEOP_DRIVE_AT_ANGLE -> SystemState.TELEOP_DRIVE_AT_ANGLE;
      case DRIVE_TO_POINT -> SystemState.DRIVE_TO_POINT;
      case AUTOPILOT -> SystemState.AUTOPILOT;
      case PATH_ON_THE_FLY -> SystemState.PATH_ON_THE_FLY;
      default -> SystemState.IDLE;
    };
  }

  // perform a desired outcome depending on our state
  private void applyStates() {
    switch (systemState) {
      default:
      case SYS_ID:
        commandtoRun = CommandtoRun.SYS_ID;
        shouldExecuteCommand = () -> true;
        break;
      case AUTO: // AUTO just automatically breaks because it is made of a completely planned set of
        // commands to follow that don't require a periodic state system
        break;
      case TELEOP_DRIVE:
        Logger.recordOutput("Subsystems/Drive/ joystick x", xJoystickInput);
        Logger.recordOutput("Subsystems/Drive/ joystick y", yJoystickInput);
        Logger.recordOutput("Subsystems/Drive/ joystick omega", omegaJoystickInput);

        joystickDrive(xJoystickInput, yJoystickInput, omegaJoystickInput);
        // calculates speeds
        // runs velocity
        break;
      case TELEOP_DRIVE_AT_ANGLE:
        joystickDriveAtAngle(xJoystickInput, yJoystickInput, joystickDriveAtAngleAngle);
        // calculates speeds and angle
        // runs velocity
        break;
      case DRIVE_TO_POINT:
        // calculates needed velo to get to state
        // gives those speeds to driveAtAngle method which then calculates the rotational component
        // runs velocity
        driveToPoint();
        break;
      case AUTOPILOT:
        // calculates the speeds necesary to reach the next autopilot setpoint
        // hands that to drive at angle
        // runs velocity
        autoPilotDriveToPoint();
        break;
      case PATH_ON_THE_FLY:
        setPathConstraintsOnTheFly();
        commandtoRun = CommandtoRun.PATH_ON_THE_FLY;
        shouldExecuteCommand = () -> true;
        break;
    }
  }

  /**
   * turns a set of overall robot relative speeds into individual module instructions for the motors
   * to then follow
   *
   * @param speeds the desired chassis speeds, in XY units of meters per second and omega units of
   *     radians per second
   */
  private void runVelocity(ChassisSpeeds speeds) {

    // calculate and optomize our given speeds
    ChassisSpeeds discreteSpeeds =
        ChassisSpeeds.discretize(
            speeds,
            0.02); // seperate individual velocity components for a given timestamp. Keep in mind
    // this can be thrown off if the speeds are later scaled
    SwerveModuleState[] setPointStates =
        kinematics.toSwerveModuleStates(
            discreteSpeeds); // turn the speeds to module states(drive motor speed and turn motor
    // angle)
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setPointStates,
        Constants.DriveConstants
            .maxSpeedMetersPerSec); // normalizes wheel velocity if any individual modules are above
    // the max speed. Keep in mind that if this is called, the
    // discretization will be innaccurate

    // logging
    Logger.recordOutput("SwerveStates/Setpoints", setPointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // set all modules to our found states. Note that we still have to optomize our wheel angle for
    // better wraparound
    // set to 3 to ignore the broken swerve module? fix once fixed
    for (int i = 0; i < 4; i++) {
      modules[i].runSwerveState(setPointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setPointStates);
  }

  /**
   * turns a set of overall speeds into individual module instructions for the motors to then follow
   * but with a max turning velo
   *
   * @param speeds the desired chassis speeds, in XY units of meters per second and omega units of
   *     radians per second
   * @param maxTurnVelocityRadiansPerSecond the max turn velo, in radians per second. It will not
   *     normalize around this number but rather act as a hard cap
   */
  public void runVelocityWithMaxTurnVelo(
      ChassisSpeeds speeds, double maxTurnVelocityRadiansPerSecond) {

    // limits our requested rotational speed to a maxium velocity before desscretizing to avoid any
    // unintentionalskew
    // simply setting a max cap and not a scale because I dont care how it gets up to this max velo
    // keep in mind that this is potential bad practice or a misinterpretation of the following
    // methods
    if (speeds.omegaRadiansPerSecond > maxTurnVelocityRadiansPerSecond) {
      speeds.omegaRadiansPerSecond = maxTurnVelocityRadiansPerSecond;
    }

    // calculate and optomize our given speeds
    ChassisSpeeds discreteSpeeds =
        ChassisSpeeds.discretize(
            speeds,
            0.02); // seperate individual velocity components for a given timestamp. Keep in mind
    // this can be thrown off if the speeds are later scaled
    SwerveModuleState[] setPointStates =
        kinematics.toSwerveModuleStates(
            discreteSpeeds); // turn the speeds to module states(drive motor speed and turn motor
    // angle)
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setPointStates,
        Constants.DriveConstants
            .maxSpeedMetersPerSec); // normalizes wheeel velocity if any individual modules are
    // above the max speed. Keep in mind that if this is called, the
    // discretization will be innaccurate

    // logging
    Logger.recordOutput("SwerveStates/Setpoints", setPointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // set all modules to our found states. Note that we still have to optomize our wheel angle for
    // better wraparound
    for (int i = 0; i < 4; i++) {
      modules[i].runSwerveState(setPointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setPointStates);
  }
  /**
   * This method takes in our various joystick inputs, and converts them into a chassis speed while
   * applying deadband then, the bot is set to run at the speeds everything should be field relative
   *
   * @param xInput the horizontal joystick input
   * @param yInput the vertical joystick input
   * @param omegaInput rotatational joystick input
   */
  public void joystickDrive(double xInput, double yInput, double omegaInput) {

    // convert the 2 seperate x & y inputs into an overall translation 2d of 1 linear speed, just
    // found as the hypotenuse of the x & y
    Translation2d linearVelocity =
        getLinearVelocityFromXY(xInput, yInput, Constants.DriveConstants.deadband);

    // Apply rotation deadband
    double omega = MathUtil.applyDeadband(omegaInput, Constants.DriveConstants.deadband);

    // Square rotation value for more precise control
    omega = Math.copySign(omega * omega, omega);

    // Convert to field relative speeds
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * getMaxLinearSpeed(),
            linearVelocity.getY() * getMaxLinearSpeed(),
            omega * getMaxAngularSpeed());
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    // set the bot to run at the chassis speeds
    runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds, isFlipped ? getRotation().plus(new Rotation2d(Math.PI)) : getRotation()));
  }
  /**
   * sets the bot to drive at any given x & y input, but stays at a given angle called with
   * joysticks providing the x and y speeds
   *
   * <p>includes field relative flipping
   *
   * @param xInput horizontal speed of the bot
   * @param yInput vertical speed of the bot
   * @param angle desired angle to lock at
   */
  public void joystickDriveAtAngle(double xInput, double yInput, Rotation2d angle) {

    System.out.println("running angle");
    // convert the 2 seperate x & y inputs into an overall translation 2d of 1 linear speed, just
    // found as the hypotenuse of the x & y
    Translation2d linearVelocity =
        getLinearVelocityFromXY(xInput, yInput, Constants.DriveConstants.deadband);

    // calculate the angle with our profiled pid controller
    double omega = angleController.calculate(getRotation().getRadians(), angle.getRadians());

    Logger.recordOutput("Subsystems/Drive/ angle pid", omega);
    Logger.recordOutput("Subsystems/Drive/ angle", angle);
    // convert to field relative speeds
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * getMaxLinearSpeed(),
            linearVelocity.getY() * getMaxLinearSpeed(),
            omega);
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    // set the bot to run at the chassis speeds
    runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds, isFlipped ? getRotation().plus(new Rotation2d(Math.PI)) : getRotation()));
  }
  /**
   * sets the bot to drive at any given x & y input, but stays at a given angle XY called with
   * external control
   *
   * <p>no flipping because that seems to break in sim, am unsure how this will work on a real field
   *
   * @param xInput horizontal speed of the bot
   * @param yInput vertical speed of the bot
   * @param angle desired angle to lock at
   */
  private void driveAtAngle(double xInput, double yInput, Rotation2d angle) {

    System.out.println("running angle");
    // convert the 2 seperate x & y inputs into an overall translation 2d of 1 linear speed, just
    // found as the hypotenuse of the x & y
    Translation2d linearVelocity = getLinearVelocityFromXY(xInput, yInput);

    // calculate the angle with our profiled pid controller
    double omega = angleController.calculate(getRotation().getRadians(), angle.getRadians());

    Logger.recordOutput("Subsystems/Drive/ angle pid", omega);
    Logger.recordOutput("Subsystems/Drive/ angle", angle);
    // convert to field relative speeds
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * getMaxLinearSpeed(),
            linearVelocity.getY() * getMaxLinearSpeed(),
            omega);

    // set the bot to run at the chassis speeds
    runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation()));
  }

  /**
   * this is the same as the other drive at angle method, but with an inbuilt max turn speed sets
   * the bot to drive at any given x & y input, but stays at a given angle
   *
   * <p>the x and y speeds are set by external control
   *
   * <p>not alliance flipping because that seems to break the bot in sim
   *
   * @param xInput horizontal speed of the bot
   * @param yInput vertical speed of the bot
   * @param angle desired angle to lock at
   * @param maxTurnVelo the max omega speed of the bot, in radians per second
   */
  private void driveAtAngle(double xInput, double yInput, Rotation2d angle, double maxTurnVelo) {
    // convert the 2 seperate x & y inputs into an overall translation 2d of 1 linear speed, just
    // found as the hypotenuse of the x & y
    Translation2d linearVelocity = getLinearVelocityFromXY(xInput, yInput);

    // calculate the angle with our profiled pid controller
    double omega = angleController.calculate(getRotation().getRadians(), angle.getRadians());

    // convert to field relative speeds
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * getMaxLinearSpeed(),
            linearVelocity.getY() * getMaxLinearSpeed(),
            omega);

    // set the bot to the chassis speeds, but use the turn limited method
    runVelocityWithMaxTurnVelo(
        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation()), maxTurnVelo);
  }

  /**
   * This sets the bot to drive straight to a desired pose It does this by calculating our needed
   * velocities to get there, then applying that to drive at angle
   */
  private void driveToPoint() {

    // difference between our desired pose and our current one
    var translationToDesiredPoint =
        driveToPointPose.getTranslation().minus(getPose().getTranslation());
    var linearDistance = translationToDesiredPoint.getNorm();

    // if we are a certain distance away from the target, calculate and later add additional speed
    // to counteract static friction
    // this method was largely copied from jack in the bot, and I am unsure why they dont do an
    // entire feed forward calc here
    // my best bet is because this isn't actually finding the voltage to set a motor to, but rather
    // the overall speed of the bot,
    // feed forward isn't applicable here, as that calculates the voltages necesary to get to a
    // speed, instead the static friction is
    // multiplied by max speed, in essence setting the bot to a slightly higher speed to beat
    // friction. If feed forward were to be applied,
    // then that would likely return too high a value, as we only want a small nudge
    var frictionConstant = 0.0;
    if (linearDistance >= Units.inchesToMeters(0.5)) {
      frictionConstant =
          Constants.DriveConstants.driveToPointStaticFrictionConstant
              * Constants.DriveConstants.maxSpeedMetersPerSec;
    }

    // the direction our linear speed needs to go
    var directionOfTravel = translationToDesiredPoint.getAngle();

    // calculate our needed velo
    var velocityOutput = 0.0;
    if (DriverStation.isAutonomous()) {
      velocityOutput =
          Math.min(
              Math.abs(autoDriveToPointController.calculate(linearDistance, 0)) + frictionConstant,
              maxVelocityOutputForDriveToPoint);
      // breaking down the math:
      // the current state of the pid is set to the distance from the wanted state
      // the desired output is a distance of 0
      // so literally we are calculating the output necesary to get our distance to 0 + friction
      // constant
      // this is an absolute value because we only care about speed right now, not direction

      // this is then compared to the maximum allowed velocity and if it is higher, the speed will
      // simply be set to the maximum velocity, otherwise
      // it will be set the the calculated speed
    } else {
      velocityOutput =
          Math.min(
              Math.abs(teleopDriveToPointController.calculate(linearDistance, 0))
                  + frictionConstant,
              maxVelocityOutputForDriveToPoint);
      // breaking down the math:
      // the current state of the pid is set to the distance from the wanted state
      // the desired output is a distance of 0
      // so literally we are calculating the output necesary to get our distance to 0 + friction
      // constant
      // this is an absolute value because we only care about speed right now, not direction

      // this is then compared to the maximum allowed velocity and if it is higher, the speed will
      // simply be set to the maximum velocity, otherwise
      // it will be set the the calculated speed
    }

    // give our speeds field a direction and break it down into x and y pieces
    var xComponent = velocityOutput * directionOfTravel.getCos();
    var yComponent = velocityOutput * directionOfTravel.getSin();

    // logging
    Logger.recordOutput("Subsystems/Drive/DriveToPoint/xVelocitySetpoint", xComponent);
    Logger.recordOutput("Subsystems/Drive/DriveToPoint/yVelocitySetpoint", yComponent);
    Logger.recordOutput("Subsystems/Drive/DriveToPoint/velocityOutput", velocityOutput);
    Logger.recordOutput("Subsystems/Drive/DriveToPoint/linearDistance", linearDistance);
    Logger.recordOutput("Subsystems/Drive/DriveToPoint/directionOfTravel", directionOfTravel);

    // if a max turn speed has been set, use the turn limited drive the angle, otherwise use the
    // standard drive to angle
    if (Double.isNaN(maxOptionalTurnVeloRadiansPerSec)) {
      driveAtAngle(xComponent, yComponent, driveToPointPose.getRotation());
    } else {
      driveAtAngle(
          xComponent, yComponent, driveToPointPose.getRotation(), maxOptionalTurnVeloRadiansPerSec);
    }
  }
  /**
   * let autopilot handle the math for the curve and simply give the velocities and angle to
   * driveAtAngle to apply to the bot
   */
  private void autoPilotDriveToPoint() {
    APResult driveResult = autopilot.calculate(getPose(), getChassisSpeeds(), autoPilotTarget);
    driveAtAngle(
        driveResult.vx().magnitude(), driveResult.vy().magnitude(), driveResult.targetAngle());
  }

  /**
   * Factory method for the command we use to create path on the flys to points. Note that we defer
   * the command to have it construct at runtime, allowing the conditions to take effect
   *
   * @return the command to goto any given point while avoiding obstacles
   */
  private Command pathOnTheFlyCommand() {
    return Commands.defer(
        () ->
            AutoBuilder.pathfindToPose(
                    pathOntheFlyPose, pathConstraintsOnTheFly, idealEndVeloOntheFly)
                .until(shouldCancelCommandEarly)
                .finallyDo(() -> shouldExecuteCommand = () -> false),
        Set.of(this));
  }

  /**
   * The boolean is largely unimportant for the drive subsystem, but can be used as an exit
   * condition if calling these as autos is demanded
   *
   * <p>If the state is either DRIVE_TO_POINT or AUTOPILOT or PATH_ON_THE_FLY then we will calculate
   * if we have arrived. Otherwise we return false
   *
   * @return true if we are in AUTOPILOT or DRIVE_TO_POINT or PATH_ON_THE_FLY, and if we have gotten
   *     to the desired pose
   */
  private boolean cancelIfNearAndReturnTrue() {
    if ((systemState == SystemState.DRIVE_TO_POINT && !DriverStation.isAutonomous())) {
      // distance to desire pose
      var distance = driveToPointPose.getTranslation().minus(getPose().getTranslation()).getNorm();

      // logging
      Logger.recordOutput("Subsystems/Drive/DriveToPointTeleOp/distanceFromEndpoint", distance);

      // checks if at pose, comparing 0 to our distance, because we want our distance to be 0
      if (MathUtil.isNear(0.0, distance, goToPoseTranslationError)) {
        setWantedState(WantedState.TELEOP_DRIVE); // go back to teleop
        return true;
      } else {
        return false;
      }
    } else if ((systemState == SystemState.DRIVE_TO_POINT && DriverStation.isAutonomous())) {

      // distance to desire pose
      var distance = driveToPointPose.getTranslation().minus(getPose().getTranslation()).getNorm();

      // loging
      Logger.recordOutput("Subsystems/Drive/DriveToPointAuto/distanceFromEndpoint", distance);

      // checks if at pose, comparing 0 to our distance, because we want our distance to be 0
      if (MathUtil.isNear(0.0, distance, goToPoseTranslationError)) {
        setWantedState(WantedState.AUTO); // go back to auto
        return true;
      } else {
        return false;
      }
    } else if (systemState == SystemState.AUTOPILOT && DriverStation.isAutonomous()) {
      // logging
      Logger.recordOutput(
          "Subsystems/Drive/AutoPilotAuto/atEndPoint",
          autopilot.atTarget(getPose(), autoPilotTarget));

      // check if at pose
      if (autopilot.atTarget(getPose(), autoPilotTarget)) {
        setWantedState(WantedState.AUTO); // return to auto
        return true;
      } else {
        return false;
      }
    } else if (systemState == SystemState.AUTOPILOT && !DriverStation.isAutonomous()) {
      Logger.recordOutput(
          "Subsystems/Drive/AutoPilotTeleop/atEndPoint",
          autopilot.atTarget(getPose(), autoPilotTarget));

      // check if at pose
      if (autopilot.atTarget(getPose(), autoPilotTarget)) {
        setWantedState(WantedState.TELEOP_DRIVE); // return to teleop
        return true;
      } else {
        return false;
      }
    }

    if ((systemState == SystemState.PATH_ON_THE_FLY && !DriverStation.isAutonomous())) {
      // distance to desire pose
      var distance = pathOntheFlyPose.getTranslation().minus(getPose().getTranslation()).getNorm();

      // logging
      Logger.recordOutput("Subsystems/Drive/PathOnTheFlyTeleOp/distanceFromEndpoint", distance);

      // checks if at pose, comparing 0 to our distance, because we want our distance to be 0
      if (MathUtil.isNear(0.0, distance, pathOntheFlyTranslationError)) {
        setWantedState(WantedState.TELEOP_DRIVE); // go back to teleop
        shouldCancelCommandEarly = () -> true;
        shouldExecuteCommand = () -> false;
        return true;
      } else {
        return false;
      }
    } else if ((systemState == SystemState.PATH_ON_THE_FLY && DriverStation.isAutonomous())) {

      // distance to desire pose
      var distance = pathOntheFlyPose.getTranslation().minus(getPose().getTranslation()).getNorm();

      // loging
      Logger.recordOutput("Subsystems/Drive/PathOnTheFlyAuto/distanceFromEndpoint", distance);

      // checks if at pose, comparing 0 to our distance, because we want our distance to be 0
      if (MathUtil.isNear(0.0, distance, pathOntheFlyTranslationError)) {
        setWantedState(WantedState.AUTO); // go back to auto
        shouldCancelCommandEarly = () -> true;
        shouldExecuteCommand = () -> false;
        return true;
      } else {
        return false;
      }
    } else {
      return false; // if all other conditions don't apply, then we are not seeking a state based
      // pose
    }
  }

  // this exists just to prevent too many states or apply states from being crowded
  private Command runSysID() {
    return switch (sysIdtoRun) {
      case NONE -> new InstantCommand(() -> {}, this);
        // TODO: write a command for this
      case DRIVE_Wheel_Radius_Characterization -> new InstantCommand(() -> {}, this);
        // TODO: write a command for this too
      case FEED_FORWARD_Calibration -> new InstantCommand(() -> {}, this);
      case QUASISTATIC_FORWARD -> sysIdQuasistatic(SysIdRoutine.Direction.kForward);
      case QUASISTATIC_REVERSE -> sysIdQuasistatic(SysIdRoutine.Direction.kReverse);
      case DYNAMIC_FORWARD -> sysIdDynamic(SysIdRoutine.Direction.kForward);
      case DYNAMIC_REVERSE -> sysIdDynamic(SysIdRoutine.Direction.kReverse);
    };
  }

  private Command runCommandControl() {
    return switch (commandtoRun) {
      case NONE -> new InstantCommand(() -> {}, this);
      case SYS_ID -> runSysID();
      case PATH_ON_THE_FLY -> pathOnTheFlyCommand();
    };
  }
  /**
   * converts a set of x and y speeds into a singular linear velocity - should be field oriented
   *
   * @param x the horizontal speed, units are arbitrary but should be meters per second for
   *     consistency
   * @param y the vertical speed, units are arbitrary but should be meters per second for
   *     consistency
   * @return a translation 2d of the linear velocity, with the direction found through trig
   */
  private static Translation2d getLinearVelocityFromXY(double x, double y) {

    // calculate the speed
    double linearMagnitude = Math.hypot(x, y);
    // calculate the direction
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }
  /**
   * same thing as the other getLinearVeloFromXY method but applies deadband for teleop converts a
   * set of x and y speeds into a singular linear velocity - should be field oriented
   *
   * @param x the horizontal speed, units are arbitrary but should be meters per second for
   *     consistency
   * @param y the vertical speed, units are arbitrary but should be meters per second for
   *     consistency
   * @return a translation 2d of the linear velocity, with the direction found through trig
   */
  private static Translation2d getLinearVelocityFromXY(double x, double y, double deadband) {
    // Apply deadband and calculate speed
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), deadband);

    // find direction
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Runs the drive in a straight line with the specified drive output.
   *
   * @param output the desired output
   */
  private void runCharacterization(Double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** stops the bot */
  private void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * @return a null command to clear the command scheduler and halt any running commands
   */
  private Command halt() {
    return Commands.runOnce(() -> {}, this);
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void StopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] =
          Constants.DriveConstants.moduleTranslations[i].getAngle(); // sets the module angles to x
    }
    kinematics.resetHeadings(headings); // applies the headings
    stop();
  }

  /**
   * In this test, the mechanism is gradually sped-up such that the voltage corresponding to
   * acceleration is negligible (hence, "as if static")
   *
   * @param direction whether to test this forward or reverse
   * @return a command to perform the desired sysID test
   */
  private Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }
  /**
   * In this test, a constant ‘step voltage’ is given to the mechanism, so that the behavior while
   * accelerating can be determined.
   *
   * @param direction whether to perform this forward or reverse
   * @return a command to perform the desired sysID test
   */
  private Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /**
   * log and get our current swerve module states
   *
   * @return an array of all 4 swerve module states
   */
  @AutoLogOutput(key = "SwerveStates/measured")
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /**
   * getter for our current swerve module position very similar to module states, but state measures
   * drive velocity, while positions measures distance moved
   *
   * @return an array of all 4 swerve module positon
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /**
   * log and getter for our bots current robot relative speed and angle, given as a Chassisspeeds
   *
   * @return a chassisSpeeds containing our bots speed and direction
   */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates()); // uses inverse kinematics
  }

  /**
   * log and getter for the position of our drive wheel when performing our wheel characterization
   * said test is not yet written
   *
   * @return an array of drive motor information for characterizing wheel radius
   */
  public double[] getWheelRadiusCharacterizationPosition() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /**
   * log and getter for module velo when doing a ff test for the drive motor said test is not yet
   * written
   *
   * @return an overall velo that combines all of the modules velo
   */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /**
   * @return the pose of the bot
   */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * @return the rotation of the bot
   */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /**
   * @return the rotational speed of the bot in radians per second
   */
  public double getYawVelocity() {
    return gyroInputs.yawVelocityRadPerSec;
  }

  /**
   * set the bot to a pose
   *
   * @param pose the pose to set the bot to
   */
  public void setPose(Pose2d pose) {
    resetSimulationPoseCallBack.accept(pose);
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    Robotstate.getInstance().setPose(poseEstimator.getEstimatedPosition());
  }

  /**
   * @return the max XY speed the bot can travel at in meters per sec
   */
  public double getMaxLinearSpeed() {
    return Constants.DriveConstants.maxSpeedMetersPerSec;
  }

  /**
   * @return the max omega speed the bot can travel at in radians per sec
   */
  public double getMaxAngularSpeed() {
    return Constants.DriveConstants.maxSpeedMetersPerSec / Constants.DriveConstants.driveBaseRadius;
  }

  /**
   * sets the bot's wanted state should be the primary way of manipulating the drivetrain outside of
   * the class
   *
   * @param wantedState the desired state
   */
  public void setWantedState(WantedState wantedState) {
    // once we enter a command based state, reset the early cancel variable
    if (wantedState == WantedState.PATH_ON_THE_FLY || wantedState == WantedState.SYS_ID) {
      setEarlyCommandCancel(false);
    }

    if (wantedState != WantedState.PATH_ON_THE_FLY && wantedState != WantedState.SYS_ID) {
      commandtoRun = CommandtoRun.NONE;
    }

    this.wantedState = wantedState;
  }

  /**
   * sets the joystick x, this will be called in the default command
   *
   * @param x horizontal joystick input
   */
  public void setXJoystickInput(double x) {
    xJoystickInput = x;
  }

  /**
   * sets the joystick y, this will be called in the default command
   *
   * @param y vertical joystick input
   */
  public void setYJoystickInput(double y) {
    yJoystickInput = y;
  }

  /**
   * sets the joystick omega, this will be called in the default command
   *
   * @param omega the omega joystick input
   */
  public void setOmegaJoystickInput(double omega) {
    omegaJoystickInput = omega;
  }

  public void setSysIDtoRun(SysIdtoRun sysIdtoRun) {
    this.sysIdtoRun = sysIdtoRun;
  }

  /**
   * sets if we want to limit the turn speed for drive at angle
   *
   * @param speed the max speed in radians per second
   */
  public void setMaxOptionalTurnVeloRadiansPerSec(double speed) {
    maxOptionalTurnVeloRadiansPerSec = speed;
  }

  /**
   * sets our angle for teleop driving with a locked angle
   *
   * @param radians the desired angle to lock to
   */
  public void setAngleLockAngle(Rotation2d radians) {
    joystickDriveAtAngleAngle = radians;
  }

  /**
   * sets pose for drive to point
   *
   * @param pose the desired pose to drive to
   */
  public void setDriveToPointPose(Pose2d pose) {
    driveToPointPose = pose;
  }

  /**
   * sets all required parameters for autopilot parameters that are not desired to be defined can be
   * set to null and they will not be called
   *
   * <p>It is defined this way instead of with multiple setters to help avoid possible excess
   * verbosity when changing the target
   *
   * <p>even though calling the method should be simpler, the method itself is a little chunky,
   * parts of this should still be extracted into their own methods
   *
   * <p>TODO: extract the entry angle and rotation logic into their own helper methods to improve
   * readibility
   *
   * @param pose the desired pose to drive to in autopilot. If null we will use the previously
   *     defined pose or default orgin 0,0 if never defined
   * @param withEntryAngle if we want an entry angle. If true then we should have defined entry
   *     angle at some point. If false then we will beeline to pose. If null then we will keep using
   *     the current state of entry
   * @param entryAngle the angle to enter the pose from. If null we continue using the previous
   *     entry angle. If prev val is not defined then it will act as if withEntryAngle == false
   * @param endVelocity the velocity in meters/s that we would like to end at. If null we will use
   *     the previously defined velo or 0 if never defined
   * @param withRotationRadius if we want a rotation radius. If true then we should have defined
   *     rotation radius at some point. If false we begin rotating asap. If null then we will keep
   *     whatever has been set before
   * @param rotationRadiusMeters how long until we start spinning toward the desired end rotation in
   *     meters. If null we continue using the previous rotation distance. If prev val is not
   *     defined then it will act as if withRotationRadius == false
   */
  public void setAutoPilotTarget(
      Pose2d pose,
      Boolean withEntryAngle,
      Rotation2d entryAngle,
      Double endVelocity,
      Boolean withRotationRadius,
      Double rotationRadiusMeters) { // withEntryAngle is called as an object Boolean rather than a
    // boolean to allow null values to exist. Primitive booleans
    // should still be able to be passed in so there should be no
    // effect outside of the method. Same thing with endVelocity, withRotationRadius, and
    // rotationRadiusMeters

    // if we want a new pose, then set it
    if (pose != null) {
      autoPilotTarget = autoPilotTarget.withReference(pose);
    }

    // if entry angle is null, then we keep whatever entry constants had been set earlier
    if (withEntryAngle != null && withEntryAngle) {
      // we know that we want an entry angle, but check to make sure that we want to set a new one
      // or keep the old one
      if (entryAngle != null) {
        autoPilotTarget = autoPilotTarget.withEntryAngle(entryAngle);
      }
      // print if we want an entry angle but the entry angle is never defined. This wont necessarily
      // break the autopilot, but it does mean entry behavior wont work
      if (entryAngle == null && autoPilotTarget.getEntryAngle().isEmpty()) {
        System.err.println("autopilot target has entry angle, but the angle is never defined");
      }
    }
    // if we dont want an entry angle than remove that from the target
    if (withEntryAngle != null && !withEntryAngle) {
      autoPilotTarget = autoPilotTarget.withoutEntryAngle();
    }

    // if we want a new end velocity, then we define it
    if (endVelocity != null) {
      autoPilotTarget = autoPilotTarget.withVelocity(endVelocity);
    }

    // if rotationRadius is null, then we keep whatever constants had been set earlier
    if (withRotationRadius != null && withRotationRadius) {
      // we know that we want a rotation radius, but check to make sure that we want to set a new
      // one
      // or keep the old one
      if (rotationRadiusMeters != null) {
        autoPilotTarget = autoPilotTarget.withRotationRadius(Meters.of(rotationRadiusMeters));
      }
      // print if we want a rotation radius but the radius is never defined. This wont necessarily
      // break the autopilot, but it does mean that we will rotate immidietely
      if (rotationRadiusMeters == null && autoPilotTarget.getRotationRadius().isEmpty()) {
        System.err.println("autopilot target has rotation radius, but the radius is never defined");
      }
    }
    // if we dont want a rotation radius than remove that from the target
    if (withRotationRadius != null && !withRotationRadius) {

      // we use this incredibly clunky method to create a copy of our target without the radius
      // because there is no withOutRotationRadius method in APTarget
      APTarget copy = new APTarget(autoPilotTarget.getReference());
      copy.withVelocity(autoPilotTarget.getVelocity());
      if (autoPilotTarget.getEntryAngle().isPresent()) {
        copy.withEntryAngle(autoPilotTarget.getEntryAngle().get());
      }

      autoPilotTarget = copy;
    }
  }
  /**
   * sets the autopilot target to match a preconfigured APTarget
   *
   * @param autoPilotTarget the target to set to
   */
  public void setAutoPilotTarget(APTarget autoPilotTarget) {
    this.autoPilotTarget = autoPilotTarget;
  }

  /**
   * sets pose for path on the fly
   *
   * @param pose the desired pose to drive to
   */
  public void setPathOntheFlyPose(Pose2d pose) {
    pathOntheFlyPose = pose;
  }

  /**
   * sets the max xy speed during path on the fly
   *
   * @param speed the max speed in meters per second
   */
  public void setMaxTransSpeedOnTheFly(double speed) {
    maxTransSpeedMpsOnTheFly = speed;
  }

  /**
   * sets the max xy acceleration during path on the fly
   *
   * @param speed the max acceleration in meters per second^2
   */
  public void setMaxTransAccelOnTheFly(double speed) {
    maxTransAccelMpssqOnTheFly = speed;
  }

  /**
   * sets the max rotational speed during path on the fly
   *
   * @param speed the max speed in meters per second^2
   */
  public void setMaxRotSpeedOnTheFly(double speed) {
    maxRotSpeedRadPerSecOnTheFly = speed;
  }

  /**
   * sets the max rotational acceleration during path on the fly
   *
   * @param speed the max acceleration in radians per second^2
   */
  public void setMaxRotAccelOnTheFly(double speed) {
    maxRotAccelRadPerSecSqOnTheFly = speed;
  }

  /**
   * set the what speed we want to end path on the fly at generally 0 to stop at the end of the
   * path, but could be higher to chain to another path/drive to point
   *
   * @param speed the desired speed in meters per second
   */
  public void setIdealEndVeloOntheFly(double speed) {
    idealEndVeloOntheFly = speed;
  }

  /** sets the path constraints for path on the fly */
  public void setPathConstraintsOnTheFly() {
    pathConstraintsOnTheFly =
        new PathConstraints(
            maxTransSpeedMpsOnTheFly,
            maxTransAccelMpssqOnTheFly,
            maxRotSpeedRadPerSecOnTheFly,
            maxRotAccelRadPerSecSqOnTheFly);
  }

  /**
   * sets if the running command (currently just path on the fly) should end early or not
   *
   * @param should if the command should end early, true is to end
   */
  public void setEarlyCommandCancel(boolean should) {
    shouldCancelCommandEarly = () -> should;
  }

  /** Adds a new timestamped vision measurement. */
  @Override
  public void accept(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }
}
