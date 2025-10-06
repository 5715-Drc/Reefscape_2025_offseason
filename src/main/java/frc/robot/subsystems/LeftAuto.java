// package frc.robot.subsystems;

// import java.util.List;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ProxyCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.vision.Vision;
// import frc.robot.subsystems.Gripper;
// import frc.robot.subsystems.Hood;
// import frc.robot.subsystems.Elevator;
// import frc.robot.commands.ElevatorCommands.ElevatorAlgaeMove;
// import frc.robot.commands.ElevatorCommands.ElevatorCoralMove;
// import frc.robot.commands.ElevatorCommands.ElevatorCoralReady;
// import frc.robot.commands.ElevatorCommands.ElevatorOffSet;
// import frc.robot.commands.ElevatorCommands.ScoreCommandL1NET;
// import frc.robot.commands.ElevatorCommands.ScoreCommandL2L3L4;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathPlannerPath;

// public class LeftAuto extends SequentialCommandGroup {
//         private final Drive drive;

//         private PathPlannerPath path3;

//         public LeftAuto(Drive drive) {

//                 this.drive = drive;
//                 try {
//                         // Load the path you want to follow using its name in the GUI

//                         path3 = PathPlannerPath.fromPathFile("LeftPath3");

//                 } catch (Exception e) {
//                 }

//                 addCommands(

//                 new ParallelCommandGroup(
//                     new InstantCommand(() -> drive.GoToPose(new Pose2d(new Translation2d(12.109,
// 5.410),
//                                                 new Rotation2d(Math.toRadians(-46.848))))),
//                                                 new CORAL_L4.hoodPos()),

//                                                 new WaitCommand(0.1),
//                                                 new GripperMove(),
//                                                 new WaitCommand(0.1),
//                                                 new ElevatorOffSet(),

//                                 new InstantCommand(() -> drive.GoToPose(new Pose2d(new
// Translation2d(14.664, 7.320),
//                                 new Rotation2d(Math.toRadians(32.905))))));

//                                 new InstantCommand(() ->  AutoBuilder.followPath(path3));
//                                 new InstantCommand(() -> drive.GoToPose(new Pose2d(new
// Translation2d(14.118, 5.400),
//                                 new Rotation2d(Math.toRadians(54.462)))));
//                                 new CORAL_L4.hoodPosd(); // Ensure CORAL_L4 and hoodPosd() are
// correctly defined in your project
//                                 new CORAL_L4.hoodPosd();
//                                 new WaitCommand(0.1);
//                                 new GripperFeeder();
//                                 new WaitCommand(0.1);
//                                 new L4();
//                                 new WaitCommand(0.1);
//                                 new GripperOutAuto();
//                                 new WaitCommand(0.1);
//                                 new  ElevatorOffSet();

// //                                 new InstantCommand(() ->
// Drive.setPoseVision(path1.getStartingHolonomicPose().get())),
// //                                 new ParallelCommandGroup(
// //                                                 AutoBuilder.followPath(path1),
// //                                                 new ElevatorAuto()),
// //                                 new WaitUntilCommand(() -> vision.isTargetFound()),
// //                                 new WaitCommand(0.5),
// //                                 new ParallelCommandGroup(
// //                                                 new GoToPoseRightAuto(),
// //                                                 new ElevatorL4()),
// //                                 new WaitCommand(0.1),

// //                                 new ScoreCoral(),
// //                                 new ElevatorOffSet(),
// //                                 new InstantCommand(() ->
// Drive.setPoseVision(path2.getStartingHolonomicPose().get())),
// //                                 AutoBuilder.followPath(path2),
// //                                 new GripperFeeder(),
// //                                 new InstantCommand(() ->
// Drive.setPoseVision(path3.getStartingHolonomicPose().get())),
// //                                 new ParallelCommandGroup(
// //                                                 AutoBuilder.followPath(path3),
// //                                                 new ElevatorAuto()),
// //                                 new WaitUntilCommand(() -> vision.isTargetFound()),
// //                                 new WaitCommand(0.5),
// //                                 new ParallelCommandGroup(
// //                                                 new GoToPoseRightAuto(),
// //                                                 new ElevatorL4()),
// //                                 new WaitCommand(0.1),
// //                                 new ScoreCoral(),
// //                                 new ElevatorOffSet(),
// //                                 new InstantCommand(() ->
// Drive.setPoseVision(path4.getStartingHolonomicPose().get())),
// //                                 AutoBuilder.followPath(path4),
// //                                 new GripperFeeder(),
// //                                 new InstantCommand(()
// ->Drive.setPoseVision(path5.getStartingHolonomicPose().get())),
// //                                 new ParallelCommandGroup(
// //                                                 AutoBuilder.followPath(path5),
// //                                                 new ElevatorAuto()),
// //                                 new WaitUntilCommand(() -> vision.isTargetFound()),
// //                                 new WaitCommand(0.1),
// //                                 new ParallelCommandGroup(
// //                                                 new GoToPoseLeftAuto(),
// //                                                 new ElevatorL4()),
// //                                 new ScoreCoral(),
// //                                 new ElevatorOffSet());
//         }
// }
