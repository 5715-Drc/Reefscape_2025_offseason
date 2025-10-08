package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer.ElevatorState;
import frc.robot.commands.AutoCommands.OffSetAuto;
import frc.robot.commands.ElevatorCommands.ElevatorAlgaeMove;
import frc.robot.commands.ElevatorCommands.ElevatorAlgaeMoveFlipped;
import frc.robot.commands.ElevatorCommands.ElevatorCoralMove;
import frc.robot.commands.ElevatorCommands.ElevatorCoralReady;
import frc.robot.commands.ElevatorCommands.ScoreCommandL2L3L4;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.drive.Drive;

public class RedMidAuto extends SequentialCommandGroup {
  private final Drive drive;
  private final Gripper gripper;
  private final Elevator elevator;

  public RedMidAuto(Drive Drive) {
    this.drive = Drive;
    this.gripper = Gripper.getGripperInstance();
    this.elevator = Elevator.getElevatorInstance();

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
                new Pose2d(new Translation2d(11.48, 3.860), new Rotation2d(Math.toRadians(0))))),
        new ScoreCommandL2L3L4(),
        new WaitCommand(0.3),
        drive.GoToPose(
            new Pose2d(new Translation2d(10.800, 4.025), new Rotation2d(Math.toRadians(0)))),
        new WaitCommand(0.3),

        // First Algae
        new ElevatorAlgaeMove(ElevatorState.ALGAE_L2),
        new WaitUntilCommand(() -> elevator.getPosition() < ElevatorState.ALGAE_L2.elevatorPos + 1),
        drive.GoToPose(
            new Pose2d(new Translation2d(11.511, 4.025), new Rotation2d(Math.toRadians(0)))),
        new WaitUntilCommand(() -> gripper.getGripperTorque() > -70),
        drive.GoToPose(
            new Pose2d(new Translation2d(10.800, 4.025), new Rotation2d(Math.toRadians(0)))),
        drive.GoToPose(
            new Pose2d(new Translation2d(9.960, 3.085), new Rotation2d(Math.toRadians(0)))),
        new ElevatorAlgaeMoveFlipped(ElevatorState.ALGAE_NET_FLIPPED),
        new InstantCommand(() -> gripper.GripperMove(1)),
        new WaitCommand(0.1),
        new OffSetAuto(),

        // Second Algea
        drive.GoToPose(
            new Pose2d(new Translation2d(11.714, 1.699), new Rotation2d(Math.toRadians(60)))),
        new ElevatorAlgaeMove(ElevatorState.ALGAE_L3),
        new WaitUntilCommand(() -> elevator.getPosition() > ElevatorState.ALGAE_L3.elevatorPos - 1),
        drive.GoToPose(
            new Pose2d(new Translation2d(12.284, 2.682), new Rotation2d(Math.toRadians(60)))),
        new WaitUntilCommand(() -> gripper.getGripperTorque() > -70),
        drive.GoToPose(
            new Pose2d(new Translation2d(11.714, 1.699), new Rotation2d(Math.toRadians(60)))),
        new WaitCommand(0.1),
        drive.GoToPose(
            new Pose2d(new Translation2d(9.960, 1.400), new Rotation2d(Math.toRadians(0)))),
        new ElevatorAlgaeMoveFlipped(ElevatorState.ALGAE_NET_FLIPPED),
        new InstantCommand(() -> gripper.GripperMove(1)),
        new WaitCommand(0.3),
        new OffSetAuto());
  }
}
