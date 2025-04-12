package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO m_io;
  private final ModuleIOInputsAutoLogged m_inputs = new ModuleIOInputsAutoLogged();

  // Module state
  private final int m_moduleNumber;
  private SwerveModulePosition[] m_odometryPositions;

  // PID controllers
  private final PIDController m_turnPID;

  /**
   * Constructs a new {@link Module} instance.
   *
   * <p>This creates a new {@link Module} object used to run the Drive and Turn motors of each
   * Module.
   *
   * @param io {@link ModuleIO} implementation of the current robot mode.
   * @param index Module number
   */
  public Module(ModuleIO io, int moduleNumber) {
    System.out.println("[Init] Creating Module");

    // Initialize IO implementation and Module number
    m_io = io;
    m_moduleNumber = moduleNumber;

    // Initialize PID controller
    m_turnPID =
        new PIDController(DriveConstants.TURN_KP, DriveConstants.TURN_KI, DriveConstants.TURN_KD);
    // Considers min and max the same point, required for the Swerve Modules since the
    // Turn position is normalized to a range of negative pi to pi
    m_turnPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Code to be run every cycle of the robot.
   *
   * <p>Must be called in {@link Drive} periodic.
   */
  public void periodic() {
    // Calculate the positions for odometry updates
    int sampleCount = m_inputs.odometryTimestamps.length;
    m_odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) { // for each odometry  sample 
      double positionMeters = m_inputs.odometryDrivePositionsRad[i] * DriveConstants.WHEEL_RADIUS_M;
      var angle = m_inputs.odometryAbsTurnPositions[i];
      m_odometryPositions[i] = new SwerveModulePosition(positionMeters, angle); // your position based on the odometry (don't trust this a lot)
    }
  }

  /**
   * Updates the logged inputs for the Module. Must be called periodically.
   *
   * @param inputs Inputs from the auto logger.
   */
  public void updateInputs() {
    m_io.updateInputs(m_inputs); // update moduleIO.java
    Logger.processInputs("Drive/Module" + Integer.toString(m_moduleNumber), m_inputs); // put it  on the logger
  }

  /**
   * Sets the idle mode of the Drive and Turn motors.
   *
   * @param enable {@code true} to enable brake mode, {@code false} to enable coast mode.
   */
  public void enableBrakeMode(boolean enable) {
    m_io.setDriveBrakeMode(enable);
    m_io.setTurnBrakeMode(enable);
  }

  /** Stops the Drive and Turn motors. */
  public void stop() {
    m_io.setDriveVoltage(0.0);
    m_io.setTurnVoltage(0.0);
  }

  /**
   * Sets voltage of the Drive motor. The value inputed is clamped between values of -12 to 12.
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed).
   */
  public void setDriveVoltage(double volts) {
    m_io.setDriveVoltage(volts);
  }

  /**
   * Sets voltage of the Turn motor. The value inputed is clamped between values of -12 to 12.
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed).
   */
  public void setTurnVoltage(double volts) {
    m_io.setTurnVoltage(volts);
  }

  /**
   * Sets the speed of the Drive motor based on a percent scale
   *
   * <p>On a -1 to 1 Scale. -1 representing -100%, 1 representing 100%.
   *
   * @param percent -1 (full reverse speed) to 1 (full forward speed).
   */
  public void setDrivePercentSpeed(double percent) {
    m_io.setDriveVoltage(percent * 12);
  }

  /**
   * Set the speed of the Turn motor based on a percent scale.
   *
   * <p>On a -1 to 1 Scale. -1 representing -100%, 1 representing 100%.
   *
   * @param percent -1 (full reverse speed) to 1 (full forward speed).
   */
  public void setTurnPercentSpeed(double percent) {
    m_io.setTurnVoltage(percent * 12);
  }

  /**
   * Sets the velocity of the Drive motor using a PID controller.
   *
   * @param velocityRadPerSec Velocity in radians per second.
   */
  public void setDriveVelocity(double velocityRadPerSec) {
    m_io.setDriveVelocity(velocityRadPerSec);
  }

  /**
   * The current absolute Turn angle of the Module in radians, normalized to a range of negative pi
   * to pi.
   *
   * @return The current Turn angle of the Module in radians.
   */
  public Rotation2d getAngle() {
    return m_inputs.turnAbsolutePositionRad;
  }

  /**
   * @return The current Drive position of the Module in radians.
   */
  public double getPositionRad() {
    return m_inputs.drivePositionRad;
  }

  /**
   * Calculates the Drive linear displacement of the Module based on the encoder readings (angular
   * position) and the wheel radius.
   *
   * @return The current Drive position of the Module in meters.
   */
  public double getPositionMeters() {
    return m_inputs.drivePositionRad * DriveConstants.WHEEL_RADIUS_M;
  }

  /**
   * Calculates the current linear velocity of the Module based on the encoder readings (angular
   * velocity) and the wheel radius.
   *
   * @return The current Drive velocity of the Module in meters per second.
   */
  public double getVelocityMetersPerSec() {
    return m_inputs.driveVelocityRadPerSec * DriveConstants.WHEEL_RADIUS_M;
  }

  /**
   * @return The current velocity of the Drive motor in radians per second.
   */
  public double getVelocityRadPerSec() {
    return m_inputs.driveVelocityRadPerSec;
  }

  /**
   * @return The current {@link SwerveModulePosition} (Turn angle and Drive position).
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /**
   * @return The current {@link SwerveModuleState} (Turn angle and Drive velocity).
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /**
   * @return An array of the odometry timestamps processed during the robot cycle.
   */
  public double[] getOdometryTimestamps() {
    return m_inputs.odometryTimestamps;
  }

  /**
   * @return An array of {@link SwerveModulePosition} calculated during the robot cycle.
   */
  public SwerveModulePosition[] getOdometryPositions() {
    return m_odometryPositions;
  }

  /**
   * Using a PID controller, calculates the voltage of the Drive and Turn motors based on the
   * current inputed setpoint.
   *
   * @param state Desired {@link SwerveModuleState} (Desired linear speed and wheel angle).
   */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle, aka take the shortest path for wheel to reach desired
    // angle in rad (-pi,pi)
    state.optimize(getAngle());

    // Run Turn motor through a PID loop
    m_io.setTurnVoltage(m_turnPID.calculate(getAngle().getRadians(), state.angle.getRadians()));

    // Linear speed m/s into velocity rad/s
    double velocityRadPerSec = state.speedMetersPerSecond / DriveConstants.WHEEL_RADIUS_M;

    // Runs the Drive motor through the TalonFX closed loop controller
    m_io.setDriveVelocity(velocityRadPerSec);
  }

  /**
   * Sets the PID gains for the Drive motor's PID controller.
   *
   * @param kP Proportional gain value.
   * @param kI Integral gain value.
   * @param kD Derivative gain value.
   */
  public void setDrivePID(double kP, double kI, double kD) {
    m_io.setDrivePID(kP, kI, kD);
  }

  /**
   * Sets the Feedforward gains for the Drive motor's Feedforward controller.
   *
   * @param kS Static gain value.
   * @param kV Velocity gain value.
   */
  public void setDriveFF(double kS, double kV) {
    m_io.setDriveFF(kS, kV);
  }

  /**
   * Sets the PID gains for the Turn motor's PID controller.
   *
   * @param kP Proportional gain value.
   * @param kI Integral gain value.
   * @param kD Derivative gain value.
   */
  public void setTurnPID(double kP, double kI, double kD) {
    m_turnPID.setPID(kP, kI, kD);
  }

  /**
   * Locks Module orientation at 0 degrees and runs Drive motor at specified voltage.
   *
   * @param output Voltage.
   */
  public void runCharacterization(double output) {
    m_io.setDriveVoltage(output);
    m_io.setTurnVoltage(m_turnPID.calculate(getAngle().getRadians(), 0)); // Setpoint at 0 degrees
  }
}
