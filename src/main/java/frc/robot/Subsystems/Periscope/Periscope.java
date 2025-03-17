// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Periscope;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Periscope extends SubsystemBase {
  private final PeriscopeIO m_io;
  private final PeriscopeIOInputsAutoLogged m_inputs = new PeriscopeIOInputsAutoLogged();

  // Controllers
  private final ProfiledPIDController m_profiledPIDController;
  private final ElevatorFeedforward m_feedforward;
  private double m_prevSetpoint = 0.0;
  private boolean m_enablePID = true;

  /**
   * This constructs a new {@link Periscope} instance.
   *
   * <p>This creates a new Periscope {@link SubsystemBase} object with the given IO implementation
   * which determines whether the methods and inputs are initialized with the real, sim, or replay
   * code.
   *
   * @param io {@link PeriscopeIO} implementation of the current robot mode
   */
  public Periscope(PeriscopeIO io) {
    System.out.println("[Init] Creating Periscope");

    // Initialize the IO implementation
    m_io = io;

    // Initialize controllers
    m_profiledPIDController =
        new ProfiledPIDController(
            PeriscopeConstants.KP,
            PeriscopeConstants.KI,
            PeriscopeConstants.KD,
            new TrapezoidProfile.Constraints(
                PeriscopeConstants.MAX_VELOCITY_M_PER_SEC,
                PeriscopeConstants.MAX_ACCELERATION_M_PER_SEC2));
    m_feedforward =
        new ElevatorFeedforward(
            PeriscopeConstants.KS,
            PeriscopeConstants.KG,
            PeriscopeConstants.KV,
            PeriscopeConstants.KA);
    m_profiledPIDController.setGoal(0);

    // Tunable PID & Feedforward gains
    SmartDashboard.putBoolean("PIDFF_Tuning/Periscope/EnableTuning", false);
    SmartDashboard.putNumber("PIDFF_Tuning/Periscope/KP", PeriscopeConstants.KP);
    SmartDashboard.putNumber("PIDFF_Tuning/Periscope/KI", PeriscopeConstants.KI);
    SmartDashboard.putNumber("PIDFF_Tuning/Periscope/KD", PeriscopeConstants.KD);
    SmartDashboard.putNumber("PIDFF_Tuning/Periscope/KS", PeriscopeConstants.KS);
    SmartDashboard.putNumber("PIDFF_Tuning/Periscope/KG", PeriscopeConstants.KG);
    SmartDashboard.putNumber("PIDFF_Tuning/Periscope/KV", PeriscopeConstants.KV);
    SmartDashboard.putNumber("PIDFF_Tuning/Periscope/KA", PeriscopeConstants.KA);
    SmartDashboard.putNumber(
        "PIDFF_Tuning/Periscope/Max_Accel", PeriscopeConstants.MAX_ACCELERATION_M_PER_SEC2);
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    // Update and log inputs
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Periscope", m_inputs);

    if (m_inputs.isHallEffectSensorTriggered[0]
        && m_inputs.heightMeters < Units.inchesToMeters(3)) {
      this.resetPosition(0);
    }

    // The feedforward calculates over 11 volts unless the a gain is changed with the tunable
    // values. This is needed so that changes the gains doesn't have to be done every time
    var feedforwardVolts =
        m_feedforward.calculateWithVelocities(
            m_profiledPIDController.getConstraints().maxVelocity,
            m_profiledPIDController.getConstraints().maxVelocity);

    if (DriverStation.isDisabled()) {
      // Don't apply feedforward if disabled
      feedforwardVolts = 0.0;
    }

    if (m_enablePID) {
      // Apply correct feedforward calculation if it's wrong (it should remain constant since max
      // velocity doesn't change)
      if (feedforwardVolts > 1) feedforwardVolts = 0.2496000000000002;
      // Calculate voltage based on PID and Feedforward controllers
      this.setVoltage(m_profiledPIDController.calculate(m_inputs.heightMeters) + feedforwardVolts);

      // Enable and update tunable PID & Feedforward gains through SmartDashboard
      if (SmartDashboard.getBoolean("PIDFF_Tuning/Periscope/EnableTuning", false)) {
        this.updatePID();
        this.updateFF();
      }
    }
  }

  /**
   * Sets the idle mode of the Periscope motors.
   *
   * @param enable {@code true} to enable brake mode, {@code false} to enable coast mode.
   */
  public void enableBrakeMode(boolean enable) {
    m_io.enableBrakeMode(enable);
  }

  /**
   * Sets the position of the Periscope motors in meters.
   *
   * @param heightMeters New position in meters.
   */
  public void resetPosition(double height) {
    m_io.resetPosition(height);
  }

  /**
   * Sets voltage of the Periscope motors. The value inputed is clamped between values of -12 to 12.
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed).
   */
  public void setVoltage(double volts) {
    m_io.setVoltage(volts);
  }

  /**
   * Triggered means that the Periscope is at the sensor. Hall Effect works just as a magnetic limit
   * switch.
   *
   * @param index Port of the desired Hall Effect sensor to get the triggered status of.
   * @return {@code true} if the specified Hall Effect sensor triggered, {@code false} if not.
   */
  public boolean isHallEffectSensorTriggered(int index) {
    return m_inputs.isHallEffectSensorTriggered[index];
  }

  /**
   * Sets the position of the Periscope using a motion profiled PID controller.
   *
   * @param heightMeters Position of the Periscope in meters.
   */
  public void setPosition(double heightMeters) {
    // Compare new setpoint to previous to determine whether to lower acceleration or not
    var acceleration =
        (heightMeters < m_prevSetpoint)
            ? PeriscopeConstants.MAX_ACCELERATION_M_PER_SEC2 / 6
            : PeriscopeConstants.MAX_ACCELERATION_M_PER_SEC2;

    // Record and update setpoint
    m_prevSetpoint = heightMeters;
    Logger.recordOutput("Superstructure/Setpoints/PeriscopeHeight", m_prevSetpoint);
    m_profiledPIDController.setGoal(heightMeters);
    m_profiledPIDController.setConstraints(
        new TrapezoidProfile.Constraints(PeriscopeConstants.MAX_VELOCITY_M_PER_SEC, acceleration));
  }

  /**
   * Sets the position of the Periscope using a motion profiled PID controller.
   *
   * @param heightMeters Position of the Periscope in meters.
   */
  public void adjustHeight(double heightMeters) {
    // Record and update setpoint
    m_prevSetpoint = m_profiledPIDController.getGoal().position + heightMeters;
    Logger.recordOutput("Superstructure/Setpoints/PeriscopeHeight", m_prevSetpoint);
    m_profiledPIDController.setGoal(m_prevSetpoint);
  }

  /**
   * Whether or not the Periscope is at its height setpoint.
   *
   * @return {@code true} if at setpoint height, {@code false} if not
   */
  public boolean atSetpointHeight() {
    return m_profiledPIDController.atGoal();
  }

  /**
   * Sets the PID gains of the Periscope motors' {@link ProfiledPIDController}.
   *
   * @param kP Proportional gain value.
   * @param kI Integral gain value.
   * @param kD Derivative gain value.
   */
  public void setPID(double kP, double kI, double kD) {
    m_profiledPIDController.setPID(kP, kI, kD);
  }

  /**
   * Sets the Feedforward gains for the Periscope motors' Feedforward controller.
   *
   * @param kS Static gain value.
   * @param kG Gravity gain value.
   * @param kV Velocity gain value.
   * @param kA Acceleration gain value.
   */
  public void setFF(double kS, double kG, double kV, double kA) {
    m_feedforward.setKs(kS);
    m_feedforward.setKg(kG);
    m_feedforward.setKv(kV);
    m_feedforward.setKa(kA);
  }

  /**
   * Sets the maximum acceleration of the {@link ProfiledPIDController}.
   *
   * @param acceleration Maximum acceleration in m/s²
   */
  public void setMaxAcceleration(double acceleration) {
    m_profiledPIDController.setConstraints(
        new TrapezoidProfile.Constraints(PeriscopeConstants.MAX_VELOCITY_M_PER_SEC, acceleration));
  }

  /**
   * Toggle closed loop {@link ProfiledPIDController} and Feedforward and open loop voltage control.
   *
   * @param enable {@code true} for closed loop control, {@code false} for open loop.
   */
  public void enablePID(boolean enable) {
    m_enablePID = enable;
  }

  /** Update PID gains for the Periscope motors from SmartDashboard inputs. */
  private void updatePID() {
    // If any value on SmartDashboard changes, update the gains
    if (PeriscopeConstants.KP
            != SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KP", PeriscopeConstants.KP)
        || PeriscopeConstants.KI
            != SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KI", PeriscopeConstants.KI)
        || PeriscopeConstants.KD
            != SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KD", PeriscopeConstants.KD)
        || PeriscopeConstants.MAX_ACCELERATION_M_PER_SEC2
            != SmartDashboard.getNumber(
                "PIDFF_Tuning/Periscope/Max_Accel",
                PeriscopeConstants.MAX_ACCELERATION_M_PER_SEC2)) {
      PeriscopeConstants.KP =
          SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KP", PeriscopeConstants.KP);
      PeriscopeConstants.KI =
          SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KI", PeriscopeConstants.KI);
      PeriscopeConstants.KD =
          SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KD", PeriscopeConstants.KD);
      PeriscopeConstants.MAX_ACCELERATION_M_PER_SEC2 =
          SmartDashboard.getNumber(
              "PIDFF_Tuning/Periscope/Max_Accel", PeriscopeConstants.MAX_ACCELERATION_M_PER_SEC2);
      // Sets the new gains
      this.setPID(PeriscopeConstants.KP, PeriscopeConstants.KI, PeriscopeConstants.KD);
      m_profiledPIDController.setConstraints(
          new TrapezoidProfile.Constraints(
              PeriscopeConstants.MAX_VELOCITY_M_PER_SEC,
              PeriscopeConstants.MAX_ACCELERATION_M_PER_SEC2));
    }
  }

  /** Update Feedforward gains for the Periscope motors from SmartDashboard inputs. */
  private void updateFF() {
    // If any value on SmartDashboard changes, update the gains
    if (PeriscopeConstants.KS
            != SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KS", PeriscopeConstants.KS)
        || PeriscopeConstants.KG
            != SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KG", PeriscopeConstants.KG)
        || PeriscopeConstants.KV
            != SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KV", PeriscopeConstants.KV)
        || PeriscopeConstants.KA
            != SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KA", PeriscopeConstants.KA)) {
      PeriscopeConstants.KS =
          SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KS", PeriscopeConstants.KS);
      PeriscopeConstants.KG =
          SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KG", PeriscopeConstants.KG);
      PeriscopeConstants.KV =
          SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KV", PeriscopeConstants.KV);
      PeriscopeConstants.KV =
          SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KA", PeriscopeConstants.KA);
      // Sets the new gains
      this.setFF(
          PeriscopeConstants.KS,
          PeriscopeConstants.KG,
          PeriscopeConstants.KV,
          PeriscopeConstants.KA);
    }
  }
}
