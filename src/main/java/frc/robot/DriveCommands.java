package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.Drive.WantedState;
import java.util.Set;

public class DriveCommands {

  private DriveCommands() {}

  public static Command pathOnTheFly(Drive drive) {

    return Commands.defer(() -> drive.pathOnTheFlyCommand(), Set.of(drive));
  }

  public static Command completeToPose(Drive drive) {
    return Commands.defer(
        () ->
            new SequentialCommandGroup(
                drive.pathOnTheFlyCommand().withTimeout(5),
                new WaitUntilCommand(() -> !drive.cancelIfNearAndReturnFalse()),
                new InstantCommand(() -> drive.setWantedState(WantedState.DRIVE_TO_POINT))
                    .withTimeout(5),
                new WaitUntilCommand(() -> !drive.cancelIfNearAndReturnFalse())),
        Set.of(drive));
  }
}
