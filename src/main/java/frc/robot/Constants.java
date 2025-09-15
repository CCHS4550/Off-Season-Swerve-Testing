// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class VisionConstants {
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static double maxAmbiguity = 0.3; // placeholder
    public static double maxZError = 0.75; // placeholder

    public static int frontCameraCanID;

    public static double linearStdDevBaseline = 0.02;
    public static double angularStdDevBaseline = 0.06;
    public static double linearStdDevMegatag2Factor = 0.5;
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
        new double[] {
          1.0, // Camera 0
          1.0 // Camera 1
        };

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "camera_0";
    public static String camera1Name = "camera_1";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 =
        new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
    public static Transform3d robotToCamera1 =
        new Transform3d(
            -0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI)); // fill with actual camera offsets
  }

  public static final class DriveConstants {
    public static final double deadband = 0.2;
    public static final double driveToPointStaticFrictionConstant = 0.02;
    public static final double maxSpeedMetersPerSec = 4.8;
    public static final double odometryFrequency = 80.0; // Hz

    // TODO: URGENT, get physical bot constants from mechanical
    public static final double trackWidth = Units.inchesToMeters(26.5);
    public static final double wheelBase = Units.inchesToMeters(26.5);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d[] moduleTranslations =
        new Translation2d[] {
          new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
        };

    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d frontLeftOffset = Rotation2d.fromRadians(5.6393);
    public static final Rotation2d frontRightOffset = Rotation2d.fromRadians(2.348);
    public static final Rotation2d backRightOffset = Rotation2d.fromRadians(1.7729);
    public static final Rotation2d backLeftOffset = Rotation2d.fromRadians(5.965215);

    // Device CAN IDs
    public static final int pigeonCanId = 9; // TODO: URGENT, switch to a nav x

    public static final int frontRightDriveCanId = 3;
    public static final int frontLeftDriveCanId = 7;
    public static final int backRightDriveCanId = 2;
    public static final int backLeftDriveCanId = 9;

    public static final int frontRightTurnCanId = 4;
    public static final int frontLeftTurnCanId = 6;
    public static final int backRightTurnCanId = 1;
    public static final int backLeftTurnCanId = 8;

    public static final int frontRightTurnEncoder = 1;
    public static final int frontLeftTurnEncoder = 3;
    public static final int backRightTurnEncoder = 0;
    public static final int backLeftTurnEncoder = 2;

    public static final boolean frontLeftTurnInverted = true;
    public static final boolean frontRightTurnInverted = true;
    public static final boolean backLeftTurnInverted = true;
    public static final boolean backRightTurnInverted = true;

    public static final boolean frontLeftDriveInverted = true;
    public static final boolean frontRightDriveInverted = true;
    public static final boolean backLeftDriveInverted = true;
    public static final boolean backRightDriveInverted = true;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 60;
    public static final double wheelRadiusMeters = Units.inchesToMeters(2);
    public static final double driveMotorReduction = 6.12;
    // public static final double driveMotorReduction = (45.0 * 22.0) / (14.0 * 15.0);
    public static final DCMotor driveGearbox = DCMotor.getNeo550(1);

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor =
        2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
    public static final double driveEncoderVelocityFactor =
        (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

    // Drive PID configuration
    public static final double driveKp = 0.05;
    public static final double driveKd = 0.0;

    public static final double driveKs = 0.16681;
    public static final double driveKv = 2.609;
    public static final double driveKa = 0.51582;
    public static final double driveSimP = 0.01;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Turn motor configuration
    public static final int turnMotorCurrentLimit = 60;
    public static final double turnMotorReduction = 12.8;
    // public static final double turnMotorReduction = 9424.0 / 203.0;
    public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

    // Turn encoder configuration
    // TODO: URGENT, apply the gear reduction
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    // Turn PID configuration
    public static final double turnKp = 2.0;
    public static final double turnKi = 0.0;
    public static final double turnKd = 0.0;
    public static final double turnSimP = 8.0;
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

    // angle lock constants
    public static final double ANGLE_KP = 6.0;
    public static final double ANGLE_KD = 0.2;
    public static final double ANGLE_MAX_VELOCITY = 8.0;
    public static final double ANGLE_MAX_ACCELERATION = 20.0;
    public static final double FF_START_DELAY = 2.0; // Secs
    public static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    public static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    public static final double WHEEL_RADIUS_RAMP_RATE = 0.05;

    // PathPlanner configuration
    // TODO: set this to the correct values
    public static final double robotMassKg = 45;
    public static final double robotMOI = 6.883;
    public static final double wheelCOF = 1.2;
    public static final RobotConfig ppConfig =
        new RobotConfig(
            robotMassKg,
            robotMOI,
            new ModuleConfig(
                wheelRadiusMeters,
                maxSpeedMetersPerSec,
                wheelCOF,
                driveGearbox.withReduction(driveMotorReduction),
                driveMotorCurrentLimit,
                1),
            moduleTranslations);

    // information for out simulated robot
    public static final DriveTrainSimulationConfig mapleSimConfig =
        DriveTrainSimulationConfig.Default()
            .withCustomModuleTranslations(moduleTranslations)
            .withRobotMass(Kilogram.of(robotMassKg))
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(
                new SwerveModuleSimulationConfig(
                    driveGearbox,
                    turnGearbox,
                    driveMotorReduction,
                    turnMotorReduction,
                    Volts.of(0.1),
                    Volts.of(0.1),
                    Meters.of(wheelRadiusMeters),
                    KilogramSquareMeters.of(0.02),
                    wheelCOF));
  }
}
