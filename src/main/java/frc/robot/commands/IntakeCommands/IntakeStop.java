package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IntakeStop extends SequentialCommandGroup {
  private final Intake intake;
  private final Indexer indexer;
  private final Gripper gripper;

  public IntakeStop() {
    this.intake = Intake.getIntakeInstance();
    this.indexer = Indexer.getIndexerInstance();
    this.gripper = Gripper.getGripperInstance();
    addRequirements(intake, indexer, gripper);

    addCommands(new InstantCommand(() -> intake.goToPosition(0)));
    addCommands(new InstantCommand(() -> intake.intakeMove(0)));
    addCommands(new InstantCommand(() -> indexer.indexerMove(0)));
  }
}
