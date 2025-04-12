package frc.robot.Subsystems.Funnel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotStateConstants;
import org.littletonrobotics.junction.Logger;

public class Funnel extends SubsystemBase { //important to extend SubsystemBase
  private final FunnelIO m_io;
  private final FunnelIOInputsAutoLogged m_inputs = new FunnelIOInputsAutoLogged(); // if this has an error dont worry check the IO file

  // PID Controller
  private final PIDController m_PIDController;
  private boolean m_enablePID = false;

  /**
   * Constructs a new {@link Funnel} instance.
   *
   * <p>This creates a new Funnel {@link SubsystemBase} object with the given IO implementation
   * which determines whether the methods and inputs are initialized with the real, sim, or replay
   * code.
   *
   * @param io {@link FunnelIO} implementation of the current mode of the robot
   */
  public Funnel(FunnelIO io) {
    System.out.println("[Init] Creating Funnel"); // for debugging

    // Initialize the IO implementation
    m_io = io;

    /// Initialize the PID controller
    m_PIDController = new PIDController(FunnelConstants.KP, FunnelConstants.KI, FunnelConstants.KD);

    // Tunable PID gains
    SmartDashboard.putBoolean("PIDFF_Tuning/Funnel/EnableTuning", false);
    SmartDashboard.putNumber("PIDFF_Tuning/Funnel/KP", FunnelConstants.KP);
    SmartDashboard.putNumber("PIDFF_Tuning/Funnel/KI", FunnelConstants.KI);
    SmartDashboard.putNumber("PIDFF_Tuning/Funnel/KD", FunnelConstants.KD);
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    // Update and log inputs
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Funnel", m_inputs);

    // Control the Funnel through the PID controller if enabled, open loop voltage control if
    // disabled
    if (m_enablePID) {
      // Calculate voltage based on PID controller
      this.setVoltage(m_PIDController.calculate(m_inputs.velocityRadPerSec));

      // Enable and update tunable PID gains through SmartDashboard
      if (SmartDashboard.getBoolean("PIDFF_Tuning/Funnel/EnableTuning", false)) {
        this.updatePID();
      }
    }
  }

  /**
   * Sets the idle mode of the Funnel motor.
   *
   * @param enable {@code true} to enable brake mode, {@code false} to enable coast mode.
   */
  public void enableBrakeMode(boolean enable) {
    m_io.enableBrakeMode(enable);
  }

  /**
   * Sets voltage of the Funnel motor. The value inputed is clamped between values of -12 to 12.
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed).
   */
  public void setVoltage(double volts) {
    m_io.setVoltage(volts);
  }

  /**
   * Sets the speed of the Funnel motor based on a percentage.
   *
   * @param percent A value between -1 (full reverse speed) to 1 (full forward speed).
   */
  public void setPercentSpeed(double percent) {
    m_io.setVoltage(percent * RobotStateConstants.MAX_VOLTAGE);
  }

  /**
   * Sets the setpoint of the Funnel PID controller.
   *
   * @param setpoint Velocity in radians per second.
   */
  public void setVelocity(double setpoint) {
    Logger.recordOutput("Superstructure/Setpoints/FunnelVelocity", setpoint);
    m_PIDController.setSetpoint(setpoint);
  }

  /**
   * Sets the PID gains for PID controller.
   *
   * @param kP Proportional gain value.
   * @param kI Integral gain value.
   * @param kD Derivative gain value.
   */
  public void setPID(double kP, double kI, double kD) {
    m_PIDController.setPID(kP, kI, kD);
  }

  /**
   * Enable closed loop PID control for the Funnel.
   *
   * @param enable {@code true} to enable PID control, {@code false} to disable.
   */
  public void enablePID(boolean enable) {
    m_enablePID = enable;
  }

  /** Update PID gains for the Funnel from SmartDashboard inputs. */
  private void updatePID() {
    // If any value on SmartDashboard changes, update the gains
    if (FunnelConstants.KP != SmartDashboard.getNumber("PIDFF_Tuning/Funnel/KP", FunnelConstants.KP)
        || FunnelConstants.KI
            != SmartDashboard.getNumber("PIDFF_Tuning/Funnel/KI", FunnelConstants.KI)
        || FunnelConstants.KD
            != SmartDashboard.getNumber("PIDFF_Tuning/Funnel/KD", FunnelConstants.KD)) {
      FunnelConstants.KP = SmartDashboard.getNumber("PIDFF_Tuning/Funnel/KP", FunnelConstants.KP);
      FunnelConstants.KI = SmartDashboard.getNumber("PIDFF_Tuning/Funnel/KI", FunnelConstants.KI);
      FunnelConstants.KD = SmartDashboard.getNumber("PIDFF_Tuning/Funnel/KD", FunnelConstants.KD);
      // Sets the new gains
      this.setPID(FunnelConstants.KP, FunnelConstants.KI, FunnelConstants.KD);
    }
  }

  /**
   * Triggered means that the beam break is broken (an object is in between the sensor).
   *
   * @return {@code true} if the sensor has been triggered, {@code false} if not.
   */
  public boolean isBeamBreakTriggered() { // this is a beam brake 
    return m_inputs.isbeamBreakTriggered;
  }
}
