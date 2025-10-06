package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IntakeGround extends SequentialCommandGroup {
  private final Intake intake;
  private final Gripper gripper;
  private final Indexer indexer;

  public IntakeGround() {
    this.intake = Intake.getIntakeInstance();
    this.indexer = Indexer.getIndexerInstance();
    this.gripper = Gripper.getGripperInstance();
    addRequirements(intake, indexer, gripper);
    addCommands(new InstantCommand(() -> intake.goToPosition(7.87109375)));
    addCommands(new InstantCommand(() -> intake.intakeMove(-1.0)));
    addCommands(new InstantCommand(() -> indexer.indexerMove(-0.2)));
    addCommands(new WaitUntilCommand(() -> indexer.IsPressed()));
    addCommands(new InstantCommand(() -> intake.goToPosition(0)));
    addCommands(new InstantCommand(() -> intake.intakeMove(0)));
    addCommands(new InstantCommand(() -> indexer.indexerMove(0)));
    // addCommands(new ElevatorOffSet());
    // addCommands(new ElevatorCoralReady());
  }
}
