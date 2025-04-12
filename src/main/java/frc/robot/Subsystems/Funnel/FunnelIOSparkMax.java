package frc.robot.Subsystems.Funnel;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RobotStateConstants;

public class FunnelIOSparkMax implements FunnelIO {
  // Funnel motor, encoder, and configurator
  private final SparkMax m_sparkmax;
  private final RelativeEncoder m_relativeEncoder;
  private final SparkMaxConfig m_config = new SparkMaxConfig(); // check drive to see what this is, is the same thing 

  /**
   * Constructs a new {@link FunnelIOSparkMax} instance.
   *
   * <p>This creates a new {@link FunnelIO} object that uses the real NEO motor to run the Funnel
   * mechanism.
   */
  public FunnelIOSparkMax() {
    System.out.println("[Init] Creating FunnelIOSparkMax");

    // Initialize the SPARK MAX with a NEO (brushless) motor
    m_sparkmax = new SparkMax(FunnelConstants.CAN_ID, MotorType.kBrushless);

    // SPARK MAX configurations
    m_config
        .inverted(FunnelConstants.IS_INVERTED)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(FunnelConstants.CUR_LIM_A);
    // setCANTimeout arguments in miliseconds so multiply by 1000 to convert sec to milisec
    m_sparkmax.setCANTimeout(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC * 1000);

    // Initialize relative encoder from SPARK MAX
    m_relativeEncoder = m_sparkmax.getEncoder();

    // Apply configuration
    m_sparkmax.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(FunnelIOInputs inputs) {
    // Update logged inputs from the motor
    inputs.appliedVoltage = m_sparkmax.getAppliedOutput() * m_sparkmax.getBusVoltage();
    inputs.currentAmps = m_sparkmax.getOutputCurrent();
    inputs.tempCelsius = m_sparkmax.getMotorTemperature();
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_relativeEncoder.getVelocity())
            / FunnelConstants.GEAR_RATIO;
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    // Update configuration
    m_config.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    // Apply configuration
    m_sparkmax.configure(
        m_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setVoltage(double volts) {
    m_sparkmax.setVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }
}
