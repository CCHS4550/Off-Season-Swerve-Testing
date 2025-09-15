package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Robotstate;
import frc.robot.Subsystems.Drive.Gyro.GyroIO;
import frc.robot.Subsystems.Drive.Gyro.GyroIOInputsAutoLogged;
import frc.robot.Subsystems.Drive.Module.*;
import frc.robot.Subsystems.Drive.Module.Module;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Util.LocalADStarAK;
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

  // values used for pathfinding to pose
  private Pose2d pathOntheFlyPose;
  private PathConstraints pathConstraintsOnTheFly;
  private double maxTransSpeedMpsOnTheFly = 20.0;
  private double maxTransAccelMpssqOnTheFly = 30;
  private double maxRotSpeedRadPerSecOnTheFly = 6;
  private double maxRotAccelRadPerSecSqOnTheFly = 10;
  private double idealEndVeloOntheFly = 0;

  // values to use during teleop, these will be periodically set during the default command
  private double xJoystickInput = 0.0;
  private double yJoystickInput = 0.0;
  private double omegaJoystickInput = 0.0;

  // angle for teleop drive but the bot is at a fixed angle
  private Rotation2d joystickDriveAtAngleAngle = Rotation2d.fromRadians(0.0);

  // constraints for drive to point functionality
  private double maxOptionalTurnVeloRadiansPerSec = Double.NaN;
  private double maxVelocityOutputForDriveToPoint = Units.feetToMeters(10.0);

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

  // acceptable margin of error when going to a posse
  private static final double goToPoseTranslationError = Units.inchesToMeters(1);

  // potential bad practice
  // mainly used in path on the fly, nothing else uses command scheduler
  private boolean isRunningCommand =
      false; // exists in order to prevent the periodic state machine from calling the same command
  // multiple times
  private BooleanSupplier shouldCancelEarly =
      () ->
          false; // we can enable should cancel early anytime we want to stop a command from running
  // state we want drive train to be in

  // used for drive simulation
  // wont be used unless for sim
  @SuppressWarnings("unused")
  private final Consumer<Pose2d> resetSimulationPoseCallBack;

  public enum WantedState {
    SYS_ID,
    AUTO,
    TELEOP_DRIVE,
    TELEOP_DRIVE_AT_ANGLE,
    PATH_ON_THE_FLY,
    DRIVE_TO_POINT,
    IDLE
  }

  // state the drive train is in
  public enum SystemState {
    SYS_ID,
    AUTO,
    TELEOP_DRIVE,
    TELEOP_DRIVE_AT_ANGLE,
    PATH_ON_THE_FLY,
    DRIVE_TO_POINT,
    IDLE
  }

  // which sys id routine to run
  // note that sysID might also be used as a stand alone command, in which case we would have to
  // apply the isRunningCommand boolean
  // as of now not a concern as we will not be runnign sysID
  public enum SysIdtoRun {
    NONE,
    DRIVE_Wheel_Radius_Characterization,
    FEED_FORWARD_Calibration,
    QUASISTATIC_FORWARD,
    QUASISTATIC_REVERSE,
    DYNAMIC_FORWARD,
    DYNAMIC_REVERSE
  }

  // initialize our states
  private SystemState systemState = SystemState.TELEOP_DRIVE;
  private WantedState wantedState = WantedState.TELEOP_DRIVE;
  private SysIdtoRun sysIdtoRun = SysIdtoRun.NONE;

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

    // create our autobuilder for pathfinder
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
    setPathOntheFlyPose(new Pose2d(3, 3, new Rotation2d()));
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
      halt();
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

    /*
     as long as isRunningCommand is true, PATH_ON_THE_FLY will not be able to be set as the system state, in order to avoid initializing the command multiple times

     cancelIfNearAndReturnFalse checks 3 conditions, if the robot is at the desired pose, what the state is, and if we are in auto

     1. If the state is anything but DRIVE_TO_POINT or PATH_ON_THE_FLY, cancelIfNear will return false, allowing us to set PATH_ON_THE_FLY whenever we want.

     2. If the state is DRIVE_TO_POINT, the boolean is unimportant, as that state does not call the command scheduler, however it does automatically switch us back over
        to either telop or auto when we are there. NOTE: the boolean value returned here can likely be used as the end condition if this this is called as a command in auto

     3. If the state is PATH_ON_THE_FLY, when we are at our desired pose, the state is switched to teleop and the boolean is switched false, allowing us to finally call it again
        If the pathfindToPose command is canceled early by the shouldCancelEarly boolean supplier, this will return false because the state is switched to teleop, which automatically returns false,
        allowing us to then call the command again if we so wish
    */
    isRunningCommand = cancelIfNearAndReturnFalse();

    // turn the states into desired output
    applyStates();

    // for (int i = 0; i < 4; i++) {
    //   modules[i].runCharacterization(10);
    // }

    Robotstate.getInstance().updateBotPoseAndSpeeds(getPose(), getChassisSpeeds());
    Robotstate.getInstance().updateRawGyroVelo(gyroInputs.yawVelocityRadPerSec);

    Logger.recordOutput("Subsystems/Drive/ pathOntheFlyGoal", pathOntheFlyPose);
    Logger.recordOutput("Subsystems/Drive/DriveDesiredPoint", driveToPointPose);

    Logger.recordOutput("Subsystems/Drive/ isRunningCommand", isRunningCommand);
    Logger.recordOutput("Subsystems/Drive/ early cancel", shouldCancelEarly.getAsBoolean());
  }

  /**
   * sets the system state to be the same as the wanted state, but can be set to perform more
   * complex judgements on what state to goto if so desired
   *
   * @return the systemstate that our systemState variable will be set to
   */
  private SystemState handleStateTransition() {
    // if we cancel early, set the state to teleop to keep us from being stuck in an idle state and
    // reset the boolean to true
    // this should only apply if the wanted state is also PATH_ON_THE_FLY so we arent stuck in a
    // cycle of setting to teleop b/c the boolean is true(which it always is when not in path on the
    // fly)
    // wanted state stays PATH_ON_THE_FLY b/c despite the system state being set to something
    // different, the wanted state is never set to anything else until a condition
    if (shouldCancelEarly.getAsBoolean() && wantedState == WantedState.PATH_ON_THE_FLY) {
      setWantedState(WantedState.TELEOP_DRIVE);
    }

    if (wantedState != WantedState.DRIVE_TO_POINT && wantedState != WantedState.PATH_ON_THE_FLY) {
      setEarlyCancel(true);
    }

    return switch (wantedState) {
      case SYS_ID -> SystemState.SYS_ID;
      case AUTO -> SystemState.AUTO;
      case TELEOP_DRIVE -> SystemState.TELEOP_DRIVE;
      case TELEOP_DRIVE_AT_ANGLE -> SystemState.TELEOP_DRIVE_AT_ANGLE;
      case PATH_ON_THE_FLY -> SystemState.PATH_ON_THE_FLY;
      case DRIVE_TO_POINT -> SystemState.DRIVE_TO_POINT;
      default -> SystemState.IDLE;
    };
  }

  // perform a desired outcome depending on our state
  private void applyStates() {
    switch (systemState) {
      default:
      case SYS_ID:
        runSysID();
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
        driveAtAngle(xJoystickInput, yJoystickInput, joystickDriveAtAngleAngle);
        // calculates speeds and angle
        // runs velocity
        break;
      case PATH_ON_THE_FLY:
        // simply calls autobuilders built in command
        // not the most accurate, should only be used for larger movements
        // only run if not already running a path on the fly
        setPathConstraintsOnTheFly();
        if (!isRunningCommand) {
          isRunningCommand = true;
        }
        break;
      case DRIVE_TO_POINT:
        // calculates needed velo to get to state
        // gives those speeds to driveAtAngle method which then calculates the rotational component
        // runs velocity
        driveToPoint();
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
  public void runVelocity(ChassisSpeeds speeds) {

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
    for (int i = 0; i < 3; i++) {
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
   * sets the bot to drive at any given x & y input, but stays at a given angle can be called with
   * joysticks providing the x and y speeds or an external pid loop note that when called with
   * joysticks, an another overload shouldve been made to apply dead band everything should be field
   * relative
   *
   * @param xInput horizontal speed of the bot
   * @param yInput vertical speed of the bot
   * @param angle desired angle to lock at
   */
  public void driveAtAngle(double xInput, double yInput, Rotation2d angle) {

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
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    // set the bot to run at the chassis speeds
    runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds, isFlipped ? getRotation().plus(new Rotation2d(Math.PI)) : getRotation()));
  }

  /**
   * this is the same as the other drive at angle method, but with an inbuilt max turn speed sets
   * the bot to drive at any given x & y input, but stays at a given angle can be called with
   * joysticks providing the x and y speeds or an external pid loop note that when called with
   * joysticks, an another overload shouldve been made to apply dead band everything should be field
   * relative
   *
   * @param xInput horizontal speed of the bot
   * @param yInput vertical speed of the bot
   * @param angle desired angle to lock at
   * @param maxTurnVelo the max omega speed of the bot, in radians per second
   */
  public void driveAtAngle(double xInput, double yInput, Rotation2d angle, double maxTurnVelo) {
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
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    // set the bot to the chassis speeds, but use the turn limited method
    runVelocityWithMaxTurnVelo(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds, isFlipped ? getRotation().plus(new Rotation2d(Math.PI)) : getRotation()),
        maxTurnVelo);
  }

  /**
   * This sets the bot to drive straight to a desired pose It does this by calculating our needed
   * velocities to get there, then applying that to drive at angle
   */
  public void driveToPoint() {

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

  public Command pathOnTheFlyCommand() {
    return AutoBuilder.pathfindToPose(
            pathOntheFlyPose, pathConstraintsOnTheFly, idealEndVeloOntheFly)
        .until(shouldCancelEarly)
        .finallyDo(() -> isRunningCommand = false); // make sure we clear the flag
  }

  /**
   * cancelIfNearAndReturnFalse checks 3 conditions, if the robot is at the desired pose, what the
   * state is, and if we are in auto
   *
   * <p>1. If the state is anything but DRIVE_TO_POINT or PATH_ON_THE_FLY, cancelIfNear will return
   * false, allowing us to set PATH_ON_THE_FLY whenever we want.
   *
   * <p>2. If the state is DRIVE_TO_POINT, the boolean is unimportant, as that state does not call
   * the command scheduler, however it does automatically switch us back over to either telop or
   * auto when we are there. NOTE: the boolean value returned here can likely be used as the end
   * condition if this this is called as a command in auto
   *
   * <p>3. If the state is PATH_ON_THE_FLY, when we are at our desired pose, the state is switched
   * to teleop and the boolean is switched false, allowing us to finally call it again If the
   * pathfindToPose command is canceled early by the shouldCancelEarly boolean supplier, this will
   * return false because the state is switched to teleop, which automatically returns false,
   * allowing us to then call the command again if we so wish
   *
   * @return if we are in PATH_ON_THE_FLY or DRIVE_TO_POINT, and if we have gotten to the desired
   *     pose
   */
  public boolean cancelIfNearAndReturnFalse() {
    if ((systemState == SystemState.PATH_ON_THE_FLY && !DriverStation.isAutonomous())) {
      // distance to desired pose
      var distance = pathOntheFlyPose.getTranslation().minus(getPose().getTranslation()).getNorm();

      // logging
      Logger.recordOutput("Subsystems/Drive/PathOnFlyTeleOp/distanceFromEndpoint", distance);

      // checks if at the pose, comparing 0 to our distance, because we want our distance to be 0
      // increased allowance of error because path on the fly is less accurate than drive to point
      if (MathUtil.isNear(0.0, distance, goToPoseTranslationError * 3)) {
        setWantedState(
            WantedState.TELEOP_DRIVE); // go back to teleop, will also resest early cancel
        return false;
      } else {
        return true;
      }
    } else if ((systemState == SystemState.DRIVE_TO_POINT && !DriverStation.isAutonomous())) {
      // distance to desire pose
      var distance = driveToPointPose.getTranslation().minus(getPose().getTranslation()).getNorm();

      // logging
      Logger.recordOutput("Subsystems/Drive/DriveToPointTeleOp/distanceFromEndpoint", distance);

      // checks if at pose, comparing 0 to our distance, because we want our distance to be 0
      if (MathUtil.isNear(0.0, distance, goToPoseTranslationError)) {
        setWantedState(WantedState.TELEOP_DRIVE); // go back to teleop, will also reset early cancel
        return false;
      } else {
        return true;
      }
    } else if ((systemState == SystemState.DRIVE_TO_POINT && DriverStation.isAutonomous())) {
      var distance = driveToPointPose.getTranslation().minus(getPose().getTranslation()).getNorm();

      Logger.recordOutput("Subsystems/Drive/DriveToPointAuto/distanceFromEndpoint", distance);

      if (MathUtil.isNear(0.0, distance, goToPoseTranslationError)) {
        setWantedState(
            WantedState
                .AUTO); // go back to auto so new tasks can be performed, will also reset early
        // cancel
        return false;
      } else {
        return true;
      }
    } else if (systemState != SystemState.PATH_ON_THE_FLY) {
      return false; // if we are not in PATH_ON_THE_FLY, then we must not be running a command for
      // swerve drive
    } else {
      return true; // if all other conditions don't apply, then we are in the middle of a
      // pathfinding command, so return true
    }
  }

  // this exists just to prevent too many states or apply states from being crowded
  public void runSysID() {
    switch (sysIdtoRun) {
      default:
        break;
      case NONE:
        break;
        // TODO: write a command for this
      case DRIVE_Wheel_Radius_Characterization:
        break;
        // TODO: write a command for this too
      case FEED_FORWARD_Calibration:
        break;
      case QUASISTATIC_FORWARD:
        sysIdQuasistatic(SysIdRoutine.Direction.kForward);
        break;
      case QUASISTATIC_REVERSE:
        sysIdQuasistatic(SysIdRoutine.Direction.kReverse);
        break;
      case DYNAMIC_FORWARD:
        sysIdDynamic(SysIdRoutine.Direction.kForward);
        break;
      case DYNAMIC_REVERSE:
        sysIdDynamic(SysIdRoutine.Direction.kReverse);
        break;
    }
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
  public void runCharacterization(Double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** stops the bot */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * @return a null command to clear the command scheduler and halt any running commands
   */
  public Command halt() {
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
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
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
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /**
   * log and get our current swerve module states
   *
   * @return an array of all 4 swerve module states
   */
  @AutoLogOutput(key = "SwerveStates/measured")
  private SwerveModuleState[] getModuleStates() {
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
  private SwerveModulePosition[] getModulePositions() {
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
  private ChassisSpeeds getChassisSpeeds() {
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
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    Pose2d robotPose = new Pose2d(pose.getTranslation(), rawGyroRotation);
    Robotstate.getInstance().setPose(robotPose);
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

    // reset the command canceller when setting the state to path on the fly and cancel the command
    // when not
    // important note: this should be called on all ways of ending PATH_ON_THE_FLY to ensure that
    // the early cancel boolean is properly set
    if (wantedState == WantedState.PATH_ON_THE_FLY) {
      setEarlyCancel(false);
    } else {
      setEarlyCancel(true);
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
  public void setEarlyCancel(boolean should) {
    shouldCancelEarly = () -> should;
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
