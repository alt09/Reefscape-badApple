package frc.robot.Subsystems.Climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO m_io;
  private final ClimberIOInputsAutoLogged m_inputs = new ClimberIOInputsAutoLogged();

  /**
   * Constructs a new {@link Climber} instance.
   *
   * <p>This creates a new Climber {@link SubsystemBase} object with the given IO implementation
   * which determines whether the methods and inputs are initialized with the real, sim, or replay
   * code.
   *
   * @param io {@link ClimberIO} implementation of the current robot mode.
   */
  public Climber(ClimberIO io) {
    System.out.println("[Init] Creating Climber");

    // Initialize the IO implementation
    m_io = io;
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    // Update and log inputs
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Climber", m_inputs);
  }

  /**
   * Sets the idle mode of the Climber motor.
   *
   * @param enable {@code true} to enable brake mode, {@code false} to enable coast mode.
   */
  public void enableBrakeMode(boolean enable) {
    m_io.enableBrakeMode(enable);
  }

  /**
   * Sets voltage of the Climber motor. The value inputed is clamped between values of -12 to 12.
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed).
   */
  public void setVoltage(double volts) {
    m_io.setVoltage(volts);
  }
}
