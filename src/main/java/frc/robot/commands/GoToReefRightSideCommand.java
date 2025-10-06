package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Drive;
import java.util.Map;
import java.util.function.Supplier;

public class GoToReefRightSideCommand extends Command {
  private final Drive drive;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final AprilTagFieldLayout fieldLayout;
  private final Map<Integer, Pose2d[]> reefTargets;
  private Command activePathCommand;

  public GoToReefRightSideCommand(
      Drive drive,
      Supplier<Pose2d> robotPoseSupplier,
      AprilTagFieldLayout fieldLayout,
      Map<Integer, Pose2d[]> reefTargets) {

    this.drive = drive;
    this.robotPoseSupplier = robotPoseSupplier;
    this.fieldLayout = fieldLayout;
    this.reefTargets = reefTargets;

    addRequirements(drive); // make sure no two commands drive at once
  }

  @Override
  public void initialize() {
    Pose2d robotPose = robotPoseSupplier.get();

    int closestTagId = -1;
    double bestDistance = Double.MAX_VALUE;

    for (AprilTag tag : fieldLayout.getTags()) {
      if (!reefTargets.containsKey(tag.ID)) continue;
      Pose2d tagPose = tag.pose.toPose2d();
      double dist = robotPose.getTranslation().getDistance(tagPose.getTranslation());
      if (dist < bestDistance) {
        bestDistance = dist;
        closestTagId = tag.ID;
      }
    }

    if (closestTagId == -1) {
      System.out.println("No valid reef tag found!");
      return;
    }

    Pose2d targetPose = reefTargets.get(closestTagId)[0];
    System.out.println("➡️ Going to reef tag " + closestTagId + " target: " + targetPose);

    PathConstraints constraints =
        new PathConstraints(3.0, 3.0, Units.degreesToRadians(720), Units.degreesToRadians(720));

    // Build the inner path-following command
    activePathCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0);

    // Schedule it safely through the scheduler
    CommandScheduler.getInstance().schedule(activePathCommand);
  }

  @Override
  public void end(boolean interrupted) {
    if (activePathCommand != null) {
      activePathCommand.cancel();
      activePathCommand = null;
    }
  }

  @Override
  public boolean isFinished() {
    // End when inner path command finishes or was never created
    return activePathCommand == null || !activePathCommand.isScheduled();
  }
}
