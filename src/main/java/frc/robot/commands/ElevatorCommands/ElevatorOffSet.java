package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Hood;

public class ElevatorOffSet extends SequentialCommandGroup {
  private final Elevator elevator;
  private final Hood hood;
  private final Gripper gripper;

  public ElevatorOffSet() {
    this.elevator = Elevator.getElevatorInstance();
    this.hood = Hood.getHoodInstance();
    this.gripper = Gripper.getGripperInstance();
    addRequirements(elevator, hood);
    addCommands(
        new InstantCommand(
            () -> hood.goToPositionMotionMagic(RobotContainer.ElevatorState.OFFSET.hoodPos)));
    addCommands(
        new InstantCommand(
            () ->
                elevator.goToPositionMotionMagic(RobotContainer.ElevatorState.OFFSET.elevatorPos)));
    addCommands(new InstantCommand(() -> gripper.GripperMove(0)));
  }
}
