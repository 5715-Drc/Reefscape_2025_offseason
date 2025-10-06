package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer.ElevatorState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hood;

public class ElevatorCoralMove extends SequentialCommandGroup {
  private final Elevator elevator;
  private final Hood hood;

  public ElevatorCoralMove(ElevatorState state) {
    this.elevator = Elevator.getElevatorInstance();
    this.hood = Hood.getHoodInstance();
    addRequirements(elevator, hood);
    addCommands(new InstantCommand(() -> hood.goToPositionMotionMagic(state.hoodPos)));
    // addCommands(new WaitUntilCommand(() -> hood.getHoodPosition() > 2));
    addCommands(new InstantCommand(() -> elevator.goToPositionMotionMagic(state.elevatorPos)));
  }
}
