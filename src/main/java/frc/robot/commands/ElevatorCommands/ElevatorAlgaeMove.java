package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Hood;

public class ElevatorAlgaeMove extends SequentialCommandGroup {
  private final Elevator elevator;
  private final Hood hood;
  private final Gripper gripper;

  public ElevatorAlgaeMove(double hoodPoition, double elevatorPosition) {
    this.elevator = Elevator.getElevatorInstance();
    this.hood = Hood.getHoodInstance();
    this.gripper = Gripper.getGripperInstance();
    addRequirements(elevator, hood, gripper);
    addCommands(new InstantCommand(() -> hood.goToPositionMotionMagic(hoodPoition)));
    addCommands(new InstantCommand(() -> elevator.goToPositionMotionMagic(elevatorPosition)));
    addCommands(new InstantCommand(() -> gripper.GripperMove(-1)));
    addCommands(new WaitUntilCommand(() -> gripper.getGripperTorque() < -70));
    addCommands(new InstantCommand(() -> gripper.GripperMove(-0.2)));
  }
}
