package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  private static Hood HoodInstance = null;
  private TalonFX HoodMotor;
  private PositionDutyCycle positionDutyCycle;
  private DutyCycleOut dutyCycleOutForMove;
  private TalonFXConfiguration HoodConfiguration;
  double positionToRotate;
  private MotionMagicConfigs motionMagicConfiguration;
  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  private Hood() {

    HoodMotor = new TalonFX(35);

    HoodConfiguration = new TalonFXConfiguration();
    HoodConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    HoodConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    HoodConfiguration.Slot0.kS = 0.25;
    HoodConfiguration.Slot0.kP =
        4; // 5.2 // A position error of 2.5 rotations results in 12 V output
    HoodConfiguration.Slot0.kI = 0.4; // 0.01 // no output for integrated error
    HoodConfiguration.Slot0.kD = 0.2; // A velocity error of 1 rps results in 0.1 V output

    HoodConfiguration.MotorOutput.PeakForwardDutyCycle = 0.5;
    HoodConfiguration.MotorOutput.PeakReverseDutyCycle = -0.5;

    motionMagicConfiguration = HoodConfiguration.MotionMagic;
    motionMagicConfiguration.MotionMagicCruiseVelocity = 60; // Target cruise velocity of 80 rps
    motionMagicConfiguration.MotionMagicAcceleration =
        120; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfiguration.MotionMagicJerk = 1200; // Target jerk of 1600 rps/s/s (0.1 seconds)

    HoodMotor.getConfigurator().apply(HoodConfiguration);

    dutyCycleOutForMove = new DutyCycleOut(0);
    positionDutyCycle = new PositionDutyCycle(0);

    HoodMotor.clearStickyFaults();
    HoodMotor.setPosition(0);
  }

  public static Hood getHoodInstance() {
    if (HoodInstance == null) HoodInstance = new Hood();
    return HoodInstance;
  }

  public void goToPositionMotionMagic(double Goal) {
    HoodMotor.setControl(m_request.withPosition(Goal));
  }

  public double getHoodPosition() {
    return HoodMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood Position", HoodMotor.getPosition().getValueAsDouble());
  }
}
