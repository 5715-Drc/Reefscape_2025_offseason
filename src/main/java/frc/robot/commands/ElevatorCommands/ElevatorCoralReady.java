package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.ElevatorState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Hood;

public class ElevatorCoralReady extends SequentialCommandGroup {
  private final Elevator elevator;
  private final Hood hood;
  private final Gripper gripper;

  public ElevatorCoralReady() {
    this.elevator = Elevator.getElevatorInstance();
    this.hood = Hood.getHoodInstance();
    this.gripper = Gripper.getGripperInstance();
    addRequirements(elevator, hood, gripper);
    addCommands(new ElevatorCoralMove(ElevatorState.OFFSET));
    addCommands(new InstantCommand(() -> gripper.GripperMove(-0.5)));
    addCommands(new InstantCommand(() -> hood.goToPositionMotionMagic(0)));
    addCommands(new WaitUntilCommand(() -> hood.getHoodPosition() < 0.5));
    addCommands(new InstantCommand(() -> elevator.goToPositionMotionMagic(-0.3)));
    addCommands(new WaitUntilCommand(() -> gripper.getGripperTorque() < -50));
    addCommands(new InstantCommand(() -> gripper.GripperMove(0)));
    addCommands(new ElevatorCoralMove(RobotContainer.ElevatorState.OFFSET));
  }
}
