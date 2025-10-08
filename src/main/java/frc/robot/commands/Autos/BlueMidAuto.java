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

public class BlueMidAuto extends SequentialCommandGroup {
  private final Drive drive;
  private final Gripper gripper;
  private final Elevator elevator;

  public BlueMidAuto(Drive Drive) {
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
                new Pose2d(new Translation2d(6.039, 4.190), new Rotation2d(Math.toRadians(180))))),
        new ScoreCommandL2L3L4(),
        new WaitCommand(0.3),
        new ParallelCommandGroup(
            drive.GoToPose(
                new Pose2d(new Translation2d(6.739, 4.025), new Rotation2d(Math.toRadians(180)))),
            new WaitCommand(0.3)),

        // First Algae
        new ParallelCommandGroup(
            new ElevatorAlgaeMove(ElevatorState.ALGAE_L2),
            drive.GoToPose(
                new Pose2d(new Translation2d(6.039, 4.025), new Rotation2d(Math.toRadians(180))))),
        new WaitUntilCommand(() -> gripper.getGripperTorque() > -50),
        drive.GoToPose(
            new Pose2d(new Translation2d(6.439, 4.025), new Rotation2d(Math.toRadians(180)))),
        new ParallelCommandGroup(
            new ElevatorAlgaeMove(ElevatorState.ALGAE_NET),
            drive.GoToPose(
                new Pose2d(new Translation2d(7.540, 4.889), new Rotation2d(Math.toRadians(0))))),
        new ElevatorAlgaeMoveFlipped(ElevatorState.ALGAE_NET_FLIPPED),
        new InstantCommand(() -> gripper.GripperMove(1)),
        new WaitCommand(0.1),
        new OffSetAuto(),

        // Second Algea
        drive.GoToPose(
            new Pose2d(new Translation2d(5.435, 5.624), new Rotation2d(Math.toRadians(-120)))),
        new ElevatorAlgaeMove(ElevatorState.ALGAE_L3),
        new WaitUntilCommand(() -> elevator.getPosition() > ElevatorState.ALGAE_L3.elevatorPos - 1),
        drive.GoToPose(
            new Pose2d(new Translation2d(5.275, 5.364), new Rotation2d(Math.toRadians(-120)))),
        new WaitUntilCommand(() -> gripper.getGripperTorque() > -50),
        drive.GoToPose(
            new Pose2d(new Translation2d(5.435, 5.624), new Rotation2d(Math.toRadians(-120)))),
        new WaitCommand(0.1),
        drive.GoToPose(new Pose2d(new Translation2d(), new Rotation2d(Math.toRadians(180)))),
        new ElevatorAlgaeMoveFlipped(ElevatorState.ALGAE_NET_FLIPPED),
        new InstantCommand(() -> gripper.GripperMove(1)),
        new WaitCommand(0.3),
        new OffSetAuto());
  }
}
