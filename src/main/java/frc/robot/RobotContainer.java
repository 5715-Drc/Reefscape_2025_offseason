// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands.ElevatorAlgaeMove;
import frc.robot.commands.ElevatorCommands.ElevatorCoralMove;
import frc.robot.commands.ElevatorCommands.ElevatorCoralReady;
import frc.robot.commands.ElevatorCommands.ElevatorOffSet;
import frc.robot.commands.ElevatorCommands.ScoreCommandL1NET;
import frc.robot.commands.ElevatorCommands.ScoreCommandL2L3L4;
import frc.robot.commands.IntakeCommands.IntakeGround;
import frc.robot.commands.IntakeCommands.IntakeOffSet;
import frc.robot.commands.goToReefSideCommand;
import frc.robot.commands.goToReefSideCommand.Side;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import java.util.EnumMap;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Intake intake;
  private final Hood hood;
  private final Indexer indexer;
  private final Elevator elevator;
  private final Gripper gripper;

  public ElevatorState state = ElevatorState.OFFSET;

  private enum GamePiece {
    CORAL,
    ALGAE,
  }

  private GamePiece currentGamePiece = GamePiece.CORAL;

  private enum POVAngle {
    UP(0),
    RIGHT(90),
    DOWN(180),
    LEFT(270);

    private final int angle;

    POVAngle(int angle) {
      this.angle = angle;
    }

    public int getAngle() {
      return angle;
    }
  }

  public enum ElevatorState {
    OFFSET(0, 2.5),
    CORAL_L1(3.75, 5.2),
    CORAL_L2(8.9, 0.0),
    CORAL_L3(8.9, 9.55),
    CORAL_L4(8.9, 23.3),
    ALGAE_L2(5.8, 6.135),
    ALGAE_L3(5.8, 14.825),
    ALGAE_NET(10.0, 23.3),
    ALGAE_FLOOR(2.65, -0.5);

    public final double hoodPos;
    public final double elevatorPos;

    ElevatorState(double hoodPos, double elevatorPos) {
      this.hoodPos = hoodPos;
      this.elevatorPos = elevatorPos;
    }
  }

  private final EnumMap<GamePiece, EnumMap<POVAngle, Command>> elevatorCommandMap =
      new EnumMap<>(GamePiece.class);

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final SendableChooser<Command> autoChooserAuto;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        // Real robot, instantiate hardware IO implementations
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooserAuto = AutoBuilder.buildAutoChooser();

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putString("Selected Piece", currentGamePiece.toString());
    SmartDashboard.putData("Auto Chooser", autoChooserAuto);
    autoChooserAuto.setDefaultOption("None", null);
    autoChooserAuto.addOption("M1Rotation", new PathPlannerAuto("M1Rotation"));
    autoChooserAuto.addOption("M2Rotation", new PathPlannerAuto("M2Rotation"));
    autoChooserAuto.addOption("M3Rotation", new PathPlannerAuto("M3Rotation"));
    autoChooserAuto.addOption("Auto1", new PathPlannerAuto("Auto1"));

    // Configure the button bindings
    EnumMap<POVAngle, Command> coralMap = new EnumMap<>(POVAngle.class);
    coralMap.put(
        POVAngle.DOWN,
        new ElevatorCoralMove(ElevatorState.CORAL_L1.hoodPos, ElevatorState.CORAL_L1.elevatorPos));
    coralMap.put(
        POVAngle.RIGHT,
        new ElevatorCoralMove(ElevatorState.CORAL_L2.hoodPos, ElevatorState.CORAL_L2.elevatorPos));
    coralMap.put(
        POVAngle.LEFT,
        new ElevatorCoralMove(ElevatorState.CORAL_L3.hoodPos, ElevatorState.CORAL_L3.elevatorPos));
    coralMap.put(
        POVAngle.UP,
        new ElevatorCoralMove(ElevatorState.CORAL_L4.hoodPos, ElevatorState.CORAL_L4.elevatorPos));

    EnumMap<POVAngle, Command> algaeMap = new EnumMap<>(POVAngle.class);
    algaeMap.put(
        POVAngle.DOWN,
        new ElevatorAlgaeMove(
            ElevatorState.ALGAE_FLOOR.hoodPos, ElevatorState.ALGAE_FLOOR.elevatorPos));
    algaeMap.put(
        POVAngle.RIGHT,
        new ElevatorAlgaeMove(ElevatorState.ALGAE_L2.hoodPos, ElevatorState.ALGAE_L2.elevatorPos));
    algaeMap.put(
        POVAngle.LEFT,
        new ElevatorAlgaeMove(ElevatorState.ALGAE_L3.hoodPos, ElevatorState.ALGAE_L3.elevatorPos));
    algaeMap.put(
        POVAngle.UP,
        new ElevatorAlgaeMove(
            ElevatorState.ALGAE_NET.hoodPos, ElevatorState.ALGAE_NET.elevatorPos));

    elevatorCommandMap.put(GamePiece.CORAL, coralMap);
    elevatorCommandMap.put(GamePiece.ALGAE, algaeMap);

    configureButtonBindings();

    intake = Intake.getIntakeInstance();
    hood = Hood.getHoodInstance();
    elevator = Elevator.getElevatorInstance();
    indexer = Indexer.getIndexerInstance();
    gripper = Gripper.getGripperInstance();
  }

  public Drive getDrive() {
    return drive;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Reset gyro to 0° when B button is pressed
    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    controller
        .x()
        .onTrue(
            new goToReefSideCommand(drive::getPose, drive.kTagLayout, drive.reefTags, Side.LEFT));
    controller
        .b()
        .onTrue(
            new goToReefSideCommand(drive::getPose, drive.kTagLayout, drive.reefTags, Side.RIGHT));

    controller
        .rightBumper()
        .onTrue(
            new IntakeGround()
                .andThen(new ElevatorCoralReady().onlyIf(() -> gripper.getGripperTorque() > -30)));
    controller.leftBumper().onTrue(new IntakeOffSet());

    controller
        .back()
        .onTrue(
            new InstantCommand(
                () -> {
                  currentGamePiece =
                      (currentGamePiece == GamePiece.CORAL) ? GamePiece.ALGAE : GamePiece.CORAL;
                  SmartDashboard.putString("Selected Piece", currentGamePiece.toString());
                }));

    controller
        .y()
        .onTrue(
            new InstantCommand(
                () -> {
                  switch (state) {
                    case CORAL_L1:
                    case ALGAE_NET:
                      // Always run Command2 for these poses
                      new ScoreCommandL1NET().schedule();
                      break;

                    default:
                      new ScoreCommandL2L3L4().schedule();
                      break;
                  }
                }));

    controller.a().onTrue(new ElevatorOffSet().andThen(() -> state = ElevatorState.OFFSET));

    controller
        .pov(0)
        .onTrue(
            new InstantCommand(
                    () -> elevatorCommandMap.get(currentGamePiece).get(POVAngle.UP).schedule())
                .andThen(
                    new InstantCommand(() -> state = (ElevatorState.ALGAE_NET))
                        .onlyIf(() -> currentGamePiece == GamePiece.ALGAE)));
    controller
        .pov(90)
        .onTrue(
            new InstantCommand(
                () -> elevatorCommandMap.get(currentGamePiece).get(POVAngle.RIGHT).schedule()));
    controller
        .pov(180)
        .onTrue(
            new InstantCommand(
                    () -> elevatorCommandMap.get(currentGamePiece).get(POVAngle.DOWN).schedule())
                .andThen(
                    new InstantCommand(() -> state = ElevatorState.CORAL_L1)
                        .onlyIf(() -> currentGamePiece == GamePiece.CORAL)));
    controller
        .pov(270)
        .onTrue(
            new InstantCommand(
                () -> elevatorCommandMap.get(currentGamePiece).get(POVAngle.LEFT).schedule()));

    controller
        .a()
        .onTrue(new InstantCommand(() -> gripper.GripperMove(0.2)))
        .onFalse(new InstantCommand(() -> gripper.GripperMove(0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
