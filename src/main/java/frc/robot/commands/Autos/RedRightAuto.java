package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer.ElevatorState;
import frc.robot.commands.AutoCommands.RedRightKeepCheckingCoral;
import frc.robot.commands.ElevatorCommands.ElevatorCoralMove;
import frc.robot.commands.ElevatorCommands.ElevatorCoralReady;
import frc.robot.commands.ElevatorCommands.ScoreCommandL2L3L4;
import frc.robot.commands.IntakeCommands.IntakeGround;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.drive.Drive;

public class RedRightAuto extends SequentialCommandGroup {
  private final Drive drive;
  private final Gripper gripper;
  private final Elevator elevator;
  private final Indexer indexer;

  public RedRightAuto(Drive Drive) {
    this.drive = Drive;
    this.gripper = Gripper.getGripperInstance();
    this.elevator = Elevator.getElevatorInstance();
    this.indexer = Indexer.getIndexerInstance();

    try {
      // Load the path you want to follow using its name in the GUI
    } catch (Exception e) {
    }

    addCommands(

        // First Coral
        new ElevatorCoralReady().onlyIf(() -> gripper.getGripperTorque() > -30),
        new WaitCommand(0.2),
        new ParallelCommandGroup(
            new ElevatorCoralMove(ElevatorState.CORAL_L4),
            new WaitCommand(0.4),
            drive.GoToPose(
                new Pose2d(new Translation2d(12.429, 5.453), new Rotation2d(Math.toRadians(-60))))),
        new ScoreCommandL2L3L4(),
        new WaitCommand(0.3),
        drive.GoToPose(
            new Pose2d(new Translation2d(12.275, 5.720), new Rotation2d(Math.toRadians(0)))),
        new WaitCommand(0.3),
        new ParallelCommandGroup(
            new IntakeGround(),
            drive.GoToPose(
                new Pose2d(
                    new Translation2d(15.004, 6.890), new Rotation2d(Math.toRadians(-154))))),
        (new RedRightKeepCheckingCoral(drive).repeatedly())
            .onlyIf(() -> indexer.IsPressed() == false),
        new WaitCommand(0.2),
        new ElevatorCoralReady(),
        new ParallelCommandGroup(
            new ElevatorCoralMove(ElevatorState.CORAL_L4),
            drive.GoToPose(
                new Pose2d(
                    new Translation2d(13.983, 5.287), new Rotation2d(Math.toRadians(-120))))));
  }
}
