package frc.robot.ControlSchemes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Subsystems.Drive.Drive;
import java.util.function.DoubleSupplier;

/** how a controller interacts with the drive train */
public class DriveScheme {
  // slow mode or fast mode
  private static DoubleSupplier driveSpeedModifier = () -> 0.5;

  static Transform2d tagTransform =
      new Transform2d(
          Units.inchesToMeters(20), Units.inchesToMeters(50), Rotation2d.fromDegrees(150));
  static Transform2d tagTransform2 =
      new Transform2d(
          Units.inchesToMeters(0), Units.inchesToMeters(60), Rotation2d.fromDegrees(60));
  static Pose2d testPose =
      new Pose2d(
          Constants.VisionConstants.aprilTagLayout
              .getTagPose(20)
              .get()
              .toPose2d()
              .plus(tagTransform)
              .getTranslation(),
          Rotation2d.fromDegrees(60));
  static Pose2d testPoseOG =
      Constants.VisionConstants.aprilTagLayout.getTagPose(20).get().toPose2d().plus(tagTransform2);

  public static void configure(Drive drive, CommandXboxController controller) {
    // default command will periodically run in drive train, in this case it periodically updates
    // our joystick values
    Command driveDefaultCommand =
        new ParallelCommandGroup(
            Commands.run(
                () ->
                    drive.setXJoystickInput(
                        controller.getLeftX() * driveSpeedModifier.getAsDouble())),
            Commands.run(
                () ->
                    drive.setYJoystickInput(
                        controller.getLeftY() * driveSpeedModifier.getAsDouble())),
            Commands.run(
                () ->
                    drive.setOmegaJoystickInput(
                        controller.getRightX() * driveSpeedModifier.getAsDouble())));
    driveDefaultCommand.addRequirements(drive);
    drive.setDefaultCommand(driveDefaultCommand);

    // set button bindings
    configureButtons(controller, drive);
  }

  // sets button bindings
  private static void configureButtons(CommandXboxController controller, Drive drive) {

    // slow mode and fast mode
    // controller.rightBumper().onTrue(Commands.runOnce(() -> setFastMode()));
    // controller.rightBumper().onFalse(Commands.runOnce(() -> setSlowMode()));

    // drive to point while button is held
    // controller
    //     .a()
    //     .onTrue(new InstantCommand(() -> drive.setWantedState(WantedState.DRIVE_TO_POINT)));
    // controller.a().onFalse(Commands.runOnce(() ->
    // drive.setWantedState(WantedState.TELEOP_DRIVE)));

    // // // path on the fly while button is held
    // controller
    //     .b()
    //     .onTrue(
    //         DriveCommands.pathOnTheFly(drive)
    //             .beforeStarting(
    //                 new InstantCommand(() -> System.out.println("runnign path on the fly")))
    //             .beforeStarting(
    //                 new InstantCommand(() ->
    // drive.setWantedState(WantedState.PATH_ON_THE_FLY))));
    // controller.b().onFalse(Commands.runOnce(() ->
    // drive.setWantedState(WantedState.TELEOP_DRIVE)));

    // // drive at angle while button is held
    //     controller
    //         .x()
    //         .onTrue(new InstantCommand(() ->
    // drive.setAngleLockAngle(Rotation2d.fromDegrees(60))));
    //     controller
    //         .x()
    //         .whileTrue(Commands.runOnce(() ->
    // drive.setWantedState(WantedState.TELEOP_DRIVE_AT_ANGLE)));
    //     controller.x().onFalse(Commands.runOnce(() ->
    // drive.setWantedState(WantedState.TELEOP_DRIVE)));

    //     controller
    //         .y()
    //         .onTrue(
    //             DriveCommands.completeToPose(drive)
    //                 .beforeStarting(
    //                     new InstantCommand(() -> System.out.println("runnign path on the fly")))
    //                 .beforeStarting(new InstantCommand(() -> drive.setIdealEndVeloOntheFly(3)))
    //                 .beforeStarting(new InstantCommand(() ->
    // drive.setDriveToPointPose(testPoseOG)))
    //                 .beforeStarting(new InstantCommand(() ->
    // drive.setPathOntheFlyPose(testPose)))
    //                 .beforeStarting(() -> drive.setWantedState(WantedState.PATH_ON_THE_FLY)));
    //     controller
    //         .y()
    //         .onFalse(Commands.runOnce(() -> drive.setWantedState(WantedState.TELEOP_DRIVE),
    // drive));

    //   }

    //   // setters for fast and slow mode
    //   public static void setFastMode() {
    //     driveSpeedModifier = () -> 1.0;
    //   }

    //   public static void setSlowMode() {
    //     driveSpeedModifier = () -> 1.0;
  }
}
