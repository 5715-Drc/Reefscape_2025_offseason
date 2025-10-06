package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private static Indexer IndexerInstance = null;

  private TalonFX m_Right;
  private TalonFX m_Left;
  private TalonFXConfiguration rightConfiguration;
  private TalonFXConfiguration leftConfiguration;
  private DutyCycleOut dutyCycleOut;

  public DigitalInput m_limitSwitch;

  private Indexer() {
    m_Right = new TalonFX(45);
    m_Left = new TalonFX(44);

    m_limitSwitch = new DigitalInput(0);

    rightConfiguration = new TalonFXConfiguration();
    rightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rightConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    leftConfiguration = new TalonFXConfiguration();
    leftConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    m_Right.getConfigurator().apply(rightConfiguration);
    m_Left.getConfigurator().apply(leftConfiguration);

    m_Left.setControl(new Follower(m_Right.getDeviceID(), true));

    dutyCycleOut = new DutyCycleOut(0);
  }

  public static Indexer getIndexerInstance() {
    if (IndexerInstance == null) IndexerInstance = new Indexer();
    return IndexerInstance;
  }

  public void indexerMove(double value) {
    m_Right.setControl(dutyCycleOut.withOutput(value));
  }

  public Boolean IsPressed() {
    return m_limitSwitch.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Limit Switch", m_limitSwitch.get());
  }
}
