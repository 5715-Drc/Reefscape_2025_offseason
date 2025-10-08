package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.drive.Drive;

public class RedRightKeepCheckingCoral extends SequentialCommandGroup {
  private final Elevator elevator;
  private final Hood hood;
  private final Gripper gripper;

  public RedRightKeepCheckingCoral(Drive drive) {
    this.elevator = Elevator.getElevatorInstance();
    this.hood = Hood.getHoodInstance();
    this.gripper = Gripper.getGripperInstance();
    addRequirements(elevator, hood, gripper);
    addCommands(
        drive.GoToPose(new Pose2d(new Translation2d(14.461, 6.623), Rotation2d.fromDegrees(-154))),
        new WaitCommand(0.5),
        drive.GoToPose(new Pose2d(new Translation2d(15.004, 6.890), Rotation2d.fromDegrees(-154))));
  }
}
