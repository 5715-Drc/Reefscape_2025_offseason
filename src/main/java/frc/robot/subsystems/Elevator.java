package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private static Elevator ElevatorInstance = null;

  private TalonFX rightMotor;
  private TalonFX leftMotor;

  private TalonFXConfiguration configuration;
  private MotionMagicConfigs motionMagicConfiguration;
  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  private DutyCycleOut dutyCycleOut;
  private PositionDutyCycle positionDutyCycle;

  private double positionToRotate;

  private Elevator() {
    rightMotor = new TalonFX(21);
    leftMotor = new TalonFX(22);

    configuration = new TalonFXConfiguration();
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configuration.Slot0.kG = 0.7;
    configuration.Slot0.kS = 0.35; // 0.25
    configuration.Slot0.kV = 0.12; // 0.12 // A velocity target of 1 rps results in 0.12 V output
    configuration.Slot0.kA = 0.005; // 0.005 // An acceleration of 1 rps/s requires 0.01 V output
    configuration.Slot0.kP =
        7.35; // 5.2 // A position error of 2.5 rotations results in 12 V output
    configuration.Slot0.kI = 0.7; // 0.01 // no output for integrated error
    configuration.Slot0.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

    motionMagicConfiguration = configuration.MotionMagic;
    motionMagicConfiguration.MotionMagicCruiseVelocity = 30; // Target cruise velocity of 80 rps
    motionMagicConfiguration.MotionMagicAcceleration =
        50; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfiguration.MotionMagicJerk = 1200; // Target jerk of 1600 rps/s/s (0.1 seconds)

    rightMotor.getConfigurator().apply(configuration);
    leftMotor.getConfigurator().apply(configuration);

    leftMotor.setControl(new Follower(rightMotor.getDeviceID(), true));

    positionDutyCycle = new PositionDutyCycle(0);
    dutyCycleOut = new DutyCycleOut(0);

    rightMotor.clearStickyFaults();
    leftMotor.clearStickyFaults();

    rightMotor.setPosition(0);
    leftMotor.setPosition(0);
  }

  public static Elevator getElevatorInstance() {
    if (ElevatorInstance == null) ElevatorInstance = new Elevator();
    return ElevatorInstance;
  }

  public void goToPositionMotionMagic(double Goal) {
    positionToRotate = Goal;
    rightMotor.setControl(m_request.withPosition(Goal));
  }

  public void resetPosition() {
    rightMotor.setPosition(0.0);
    leftMotor.setPosition(0.0);
    rightMotor.setControl(m_request.withPosition(0));
  }

  public void ElevatorMove(double speed) {
    rightMotor.setControl(dutyCycleOut.withOutput(speed));
  }

  public void ElevatorStop() {
    rightMotor.setControl(dutyCycleOut.withOutput(0));
  }

  public boolean IsReachedTarget() {
    return rightMotor.getPosition().getValueAsDouble() > positionToRotate - 1
        && rightMotor.getPosition().getValueAsDouble() < positionToRotate + 1;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "Right elevator position", rightMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Left elevator position", rightMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber(
        "Right elevator Voltage", rightMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber(
        "Left elevator Voltage", leftMotor.getMotorVoltage().getValueAsDouble());
  }
}
