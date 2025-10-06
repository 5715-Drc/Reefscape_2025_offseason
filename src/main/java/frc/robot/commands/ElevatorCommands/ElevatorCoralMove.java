package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hood;

public class ElevatorCoralMove extends SequentialCommandGroup {
  private final Elevator elevator;
  private final Hood hood;

  public ElevatorCoralMove(double hoodPoition, double elevatorPosition) {
    this.elevator = Elevator.getElevatorInstance();
    this.hood = Hood.getHoodInstance();
    addRequirements(elevator, hood);
    addCommands(new InstantCommand(() -> hood.goToPositionMotionMagic(hoodPoition)));
    // addCommands(new WaitUntilCommand(() -> hood.getHoodPosition() > 2));
    addCommands(new InstantCommand(() -> elevator.goToPositionMotionMagic(elevatorPosition)));
  }
}
