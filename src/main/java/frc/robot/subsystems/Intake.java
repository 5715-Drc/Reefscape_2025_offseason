package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private static Intake IntakeInstance = null;

  private TalonFX m_Move;
  private TalonFX m_Rotation;
  private TalonFXConfiguration MoveConfiguration;
  private TalonFXConfiguration RotationConfiguration;
  private MotionMagicConfigs motionMagicConfiguration;
  private DutyCycleOut dutyCycleOut;
  private double positionToRotate;

  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  // private DigitalInput limitSwitch = new DigitalInput(0);

  private Intake() {
    m_Move = new TalonFX(14);
    m_Rotation = new TalonFX(15);

    MoveConfiguration = new TalonFXConfiguration();
    MoveConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    RotationConfiguration = new TalonFXConfiguration();
    RotationConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    RotationConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    RotationConfiguration.Slot0.kS = 0.25; // 0.25
    RotationConfiguration.Slot0.kV =
        0.12; // 0.12 // A velocity target of 1 rps results in 0.12 V output
    RotationConfiguration.Slot0.kA =
        0.05; // 0.005 // An acceleration of 1 rps/s requires 0.01 V output
    RotationConfiguration.Slot0.kP =
        5.5; // 5.2 // A position error of 2.5 rotations results in 12 V output
    RotationConfiguration.Slot0.kI = 0.01; // 0.01 // no output for integrated error
    RotationConfiguration.Slot0.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

    motionMagicConfiguration = RotationConfiguration.MotionMagic;
    motionMagicConfiguration.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 80 rps
    motionMagicConfiguration.MotionMagicAcceleration =
        30; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfiguration.MotionMagicJerk = 1200; // Target jerk of 1600 rps/s/s (0.1 seconds)

    m_Move.getConfigurator().apply(MoveConfiguration);
    m_Rotation.getConfigurator().apply(RotationConfiguration);

    m_Rotation.setPosition(0);

    m_Move.clearStickyFaults();
    m_Rotation.clearStickyFaults();

    dutyCycleOut = new DutyCycleOut(0);
  }

  public Command resetAngleCommand() {
    return new RunCommand(() -> m_Rotation.setControl(m_request.withPosition(0)))
        .ignoringDisable(true);
  }

  public static Intake getIntakeInstance() {
    if (IntakeInstance == null) IntakeInstance = new Intake();
    return IntakeInstance;
  }

  public void goToPosition(double Goal) {
    m_Rotation.setControl(m_request.withPosition(Goal));
  }

  public void intakeMove(double value) {
    m_Move.setControl(dutyCycleOut.withOutput(value));
  }

  public void intakeRotation(double value) {
    m_Rotation.setControl(dutyCycleOut.withOutput(value));
  }

  // public boolean hasCoral() {
  //   return limitSwitch.get();
  // }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Position", m_Rotation.getPosition().getValueAsDouble());
    // SmartDashboard.putBoolean("LimitSwitch", limitSwitch.get());
    // if (limitSwitch.get()) {
    //   new IntakeOffSet().schedule();
    // }
  }
}
