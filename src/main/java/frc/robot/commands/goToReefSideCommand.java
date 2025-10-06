package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Map;
import java.util.function.Supplier;

public class goToReefSideCommand extends Command {
  public enum Side {
    LEFT,
    RIGHT
  }

  private final Supplier<Pose2d> robotPoseSupplier;
  private final AprilTagFieldLayout fieldLayout;
  private final Map<Integer, Pose2d[]> reefTargets; // tagID -> {leftPose, rightPose}
  private final Side side;
  PathConstraints constraints =
      new PathConstraints(2.0, 2.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  private Command inner;

  public goToReefSideCommand(
      Supplier<Pose2d> robotPoseSupplier,
      AprilTagFieldLayout fieldLayout,
      Map<Integer, Pose2d[]> reefTargets,
      Side side) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.fieldLayout = fieldLayout;
    this.reefTargets = reefTargets;
    this.side = side;
  }

  @Override
  public void initialize() {
    Pose2d robotPose = robotPoseSupplier.get();

    // Find closest reef tag by squared distance
    double bestDistSq = Double.MAX_VALUE;
    int bestTagId = -1;

    for (AprilTag tag : fieldLayout.getTags()) {
      if (!reefTargets.containsKey(tag.ID)) continue;

      Pose2d tagPose = tag.pose.toPose2d();
      double dx = robotPose.getX() - tagPose.getX();
      double dy = robotPose.getY() - tagPose.getY();
      double distSq = dx * dx + dy * dy;

      if (distSq < bestDistSq) {
        bestDistSq = distSq;
        bestTagId = tag.ID;
      }
    }

    if (bestTagId < 0) {
      inner = null;
      return;
    }

    // Pick left or right pose based on button/side
    Pose2d[] pair = reefTargets.get(bestTagId);
    Pose2d target = (side == Side.LEFT) ? pair[0] : pair[1];

    inner = AutoBuilder.pathfindToPose(target, constraints, 0);
    // CommandScheduler.getInstance().schedule(inner);
  }

  @Override
  public void execute() {
    // nothing, inner runs itself
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("end ------");
    CommandScheduler.getInstance().schedule(inner);
    // if (inner != null && CommandScheduler.getInstance().isScheduled(inner)) {
    //   inner.cancel();
    // }
    // inner = null;
  }

  @Override
  public boolean isFinished() {
    return true;
    // return inner == null || !CommandScheduler.getInstance().isScheduled(inner);
  }
}
