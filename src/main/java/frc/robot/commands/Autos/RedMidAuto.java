package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer.ElevatorState;
import frc.robot.commands.*;
import frc.robot.commands.ElevatorCommands.ElevatorCoralMove;
import frc.robot.commands.ElevatorCommands.ElevatorCoralReady;
import frc.robot.commands.ElevatorCommands.ElevatorOffSet;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.drive.Drive;

public class RedMidAuto extends SequentialCommandGroup {
  private final Drive drive;
  private final Gripper gripper;

  public RedMidAuto(Drive drive) {
    this.drive = drive;
    this.gripper = Gripper.getGripperInstance();
    addRequirements(drive);

    try {

    } catch (Exception e) {
    }

    addCommands(
        new ElevatorOffSet(),
        new WaitCommand(0.2),
        new ElevatorCoralReady().onlyIf(() -> gripper.getGripperTorque() > -30),
        new ParallelCommandGroup(
            new ElevatorCoralMove(ElevatorState.CORAL_L2),
            drive.GoToPose(new Pose2d(11.516, 3.865, Rotation2d.fromDegrees(0)))));
  }
}
