package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
  private static Gripper GripperInstance = null;
  private TalonFX CoralGripperMotor;
  private DutyCycleOut dutyCycleOutForMove;
  private TalonFXConfiguration CoralConfiguration;
  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  private Gripper() {

    CoralGripperMotor = new TalonFX(18);

    CoralConfiguration = new TalonFXConfiguration();
    CoralConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    CoralConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    CoralGripperMotor.getConfigurator().apply(CoralConfiguration);

    dutyCycleOutForMove = new DutyCycleOut(0);
  }

  public void GripperMove(double speed) {
    CoralGripperMotor.setControl(dutyCycleOutForMove.withOutput(speed));
  }

  public void StopGripper() {
    CoralGripperMotor.setControl(dutyCycleOutForMove.withOutput(0));
  }

  public void RotationGripperMove(double speed) {
    CoralGripperMotor.setControl(dutyCycleOutForMove.withOutput(speed));
  }

  public void StopRotationGripper() {
    CoralGripperMotor.setControl(dutyCycleOutForMove.withOutput(0));
  }

  public static Gripper getGripperInstance() {
    if (GripperInstance == null) GripperInstance = new Gripper();
    return GripperInstance;
  }

  public double getGripperTorque() {
    return CoralGripperMotor.getTorqueCurrent().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Torque", CoralGripperMotor.getTorqueCurrent().getValueAsDouble());
  }
}
