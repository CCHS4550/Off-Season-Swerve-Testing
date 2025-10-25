package frc.robot;

import static frc.robot.Constants.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.ControlSchemes.DriveScheme;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.Gyro.GyroIO;
import frc.robot.Subsystems.Drive.Gyro.GyroIONavX;
import frc.robot.Subsystems.Drive.Gyro.GyroIOSim;
import frc.robot.Subsystems.Drive.Module.ModuleIO;
import frc.robot.Subsystems.Drive.Module.ModuleIOSim;
import frc.robot.Subsystems.Drive.Module.ModuleIOSpark;
import frc.robot.Subsystems.QuestNav.QuestNav;
import frc.robot.Subsystems.QuestNav.QuestNavIO;
import frc.robot.Subsystems.QuestNav.QuestNavIOQuest;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionIO;
import frc.robot.Subsystems.Vision.VisionIOSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  // subclasses of the robot
  public final Drive drive;
  private final QuestNav questNav;
  private final Vision vision;

  // drive sim
  private SwerveDriveSimulation driveSimulation = null;

  // controller used
  CommandXboxController primaryController = new CommandXboxController(0);

  // autochooser
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3),
                (pose) -> {});

        questNav =
            new QuestNav(
                drive,
                new QuestNavIOQuest(
                    new Transform3d())); // the transfrom 3d is blank rn, find its real value and
        // put it in constants

        Robotstate.getInstance().addListener(questNav);

        vision = new Vision(questNav, new VisionIO() {}, new VisionIO() {});

        // configure control schemes
        DriveScheme.configure(drive, primaryController);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        // create a maple-sim swerve drive simulation instance
        this.driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(2, 7, new Rotation2d()));
        // add the simulated drivetrain to the simulation field
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        // create the simulated drive
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);

        questNav = new QuestNav(drive, new QuestNavIO() {});

        vision =
            new Vision(
                questNav,
                new VisionIOSim(
                    camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOSim(
                    camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));

        DriveScheme.configure(drive, primaryController);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {});

        questNav = new QuestNav(drive, new QuestNavIO() {});

        vision = new Vision(questNav, new VisionIO() {}, new VisionIO() {});

        break;
    }

    NamedCommands.registerCommand(
        "test print", new RunCommand(() -> System.out.println("the test auto is running")));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addDefaultOption("test auto", new PathPlannerAuto("test auto"));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** functions to be used to update and set the maple sim physics field */
  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    drive.setPose(new Pose2d(2, 7, new Rotation2d()));

    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
