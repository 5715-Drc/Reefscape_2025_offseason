package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Hood;

public class ScoreCommandL2L3L4 extends SequentialCommandGroup {
  private final Elevator elevator;
  private final Hood hood;
  private final Gripper gripper;

  public ScoreCommandL2L3L4() {
    this.elevator = Elevator.getElevatorInstance();
    this.hood = Hood.getHoodInstance();
    this.gripper = Gripper.getGripperInstance();
    addRequirements(elevator, hood, gripper);
    addCommands(new InstantCommand(() -> hood.goToPositionMotionMagic(7)));
    addCommands(new WaitUntilCommand(() -> hood.getHoodPosition() < 7.5));
    addCommands(new InstantCommand(() -> gripper.GripperMove(0.15)));
  }
}
