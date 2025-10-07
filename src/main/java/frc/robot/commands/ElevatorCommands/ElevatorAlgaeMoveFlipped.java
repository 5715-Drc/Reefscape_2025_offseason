package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer.ElevatorState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Hood;

public class ElevatorAlgaeMoveFlipped extends SequentialCommandGroup {
  private final Elevator elevator;
  private final Hood hood;
  private final Gripper gripper;

  public ElevatorAlgaeMoveFlipped(ElevatorState state) {
    this.elevator = Elevator.getElevatorInstance();
    this.hood = Hood.getHoodInstance();
    this.gripper = Gripper.getGripperInstance();
    addRequirements(elevator, hood, gripper);
    addCommands(new InstantCommand(() -> elevator.goToPositionMotionMagic(state.elevatorPos)));
    addCommands(new WaitUntilCommand(() -> elevator.getPosition() > 23.5));
    addCommands(new InstantCommand(() -> hood.goToPositionMotionMagic(state.hoodPos)));
    addCommands(new WaitUntilCommand(() -> hood.getHoodPosition() > 14));
  }
}
