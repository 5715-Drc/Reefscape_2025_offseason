package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.ElevatorCommands.ElevatorCoralMove;
import frc.robot.commands.ElevatorCommands.ElevatorCoralReady;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IntakeAdjust extends SequentialCommandGroup {
  private final Intake intake;
  private final Indexer indexer;
  private final Gripper gripper;

  public IntakeAdjust() {
    this.intake = Intake.getIntakeInstance();
    this.indexer = Indexer.getIndexerInstance();
    this.gripper = Gripper.getGripperInstance();
    addRequirements(intake, indexer, gripper);

    addCommands(
        new ElevatorCoralMove(
            RobotContainer.ElevatorState.OFFSET.hoodPos,
            RobotContainer.ElevatorState.OFFSET.elevatorPos));
    addCommands(new ElevatorCoralReady());
  }
}
