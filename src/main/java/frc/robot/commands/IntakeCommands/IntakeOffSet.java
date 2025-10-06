package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IntakeOffSet extends SequentialCommandGroup {
  private final Intake intake;
  private final Indexer indexer;

  public IntakeOffSet() {
    this.intake = Intake.getIntakeInstance();
    this.indexer = Indexer.getIndexerInstance();
    addRequirements(intake, indexer);
    addCommands(new InstantCommand(() -> intake.goToPosition(0)));
    addCommands(new InstantCommand(() -> intake.intakeMove(0)));
    addCommands(new InstantCommand(() -> indexer.indexerMove(0)));
  }
}
