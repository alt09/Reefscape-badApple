package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.RobotStateConstants;

/** ModuleIO implementation for the simulated mode of the robot */
public class ModuleIOSim implements ModuleIO {
  // Flywheel simulations
  private final DCMotorSim m_driveSim;
  private final DCMotorSim m_turnSim;

  // Motor voltages
  private double m_driveAppliedVolts = 0.0;
  private double m_turnAppliedVolts = 0.0;

  // PID & Feedforward controllers
  private final PIDController m_driveController;
  private SimpleMotorFeedforward m_driveFeedforward;
  private double m_driveSetpointRadPerSec = 0.0;

  /**
   * Constructs a new {@link ModuleIOSim} instance.
   *
   * <p>This creates a new {@link ModuleIO} object that uses the simulated versions of the KrakenX60
   * and NEO motors to run the Drive and Turn of the simulated Module.
   */
  public ModuleIOSim() {
    System.out.println("[Init] Creating ModuleIOSim");

    // Initialize simulated motors
    m_driveSim =
        new DCMotorSim( // is the simulation of a DC motor  aka a krakenX60 or a neo or neo 550(the small neo)
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),// just one kraken for the motortype
                DriveConstants.DRIVE_MOI_KG_M2, // ask  design bruh 
                DriveConstants.DRIVE_GEAR_RATIO),// ask design bruh
            DCMotor.getKrakenX60(1)); // the gearing of just one kraken 
    m_turnSim =
        new DCMotorSim( // now is a turn motor aka a neo
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNEO(1), // just one neo for the motortype
                 DriveConstants.TURN_MOI_KG_M2, // ask design bruh
                  DriveConstants.TURN_GEAR_RATIO), // ask design bruh 
            DCMotor.getNEO(1)); // the gearing of just one neo

    // Initialize PID & Feedforward controllers
    m_driveController =
        new PIDController(
            DriveConstants.DRIVE_KP_SIM, DriveConstants.DRIVE_KI_SIM, DriveConstants.DRIVE_KD_SIM); // PID bruh GL GG
    m_driveFeedforward =
        new SimpleMotorFeedforward(DriveConstants.DRIVE_KS_SIM, DriveConstants.DRIVE_KV_SIM);// PID bruh GL GG
  }

  @Override// if this is red in all the way to the top extending ModuleIO
  public void updateInputs(ModuleIOInputs inputs) {
    // Calculate and apply next output voltage from the PID and Feedforward controller
    m_driveAppliedVolts =
        m_driveController.calculate(inputs.driveVelocityRadPerSec, m_driveSetpointRadPerSec) 
            + m_driveFeedforward.calculate(m_driveSetpointRadPerSec); // adding the feedforward to be more precise 
    this.setDriveVoltage(m_driveAppliedVolts); // WOW guess what this does 

    // Update simulated motors
    m_driveSim.update(RobotStateConstants.LOOP_PERIODIC_SEC); //updates this every 0.02 sec because this is what the roborio upate rate is 
    m_turnSim.update(RobotStateConstants.LOOP_PERIODIC_SEC); // maybe idk 

    // Update logged Drive motor inputs from the simulated flywheel system
    inputs.driveIsConnected = true; // WOW
    inputs.driveAppliedVoltage = m_driveAppliedVolts; // LOL
    inputs.driveCurrentAmps = Math.abs(m_driveSim.getCurrentDrawAmps()); // OMG
    inputs.drivePositionRad = m_driveSim.getAngularPositionRad(); // absolute cinema
    inputs.driveVelocityRadPerSec = m_driveSim.getAngularVelocityRadPerSec(); //chicken joykey 

    // Update logged Turn motor inputs from the simulated flywheel system
    inputs.absoluteEncoderIsConnected = true;
    inputs.turnAppliedVoltage = m_turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(m_turnSim.getCurrentDrawAmps()); // just make everything positive 
    inputs.turnAbsolutePositionRad =
        Rotation2d.fromRadians(MathUtil.angleModulus(m_turnSim.getAngularPositionRad()));
    inputs.turnVelocityRadPerSec = m_turnSim.getAngularVelocityRadPerSec();

    // Update odometry inputs
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryAbsTurnPositions = new Rotation2d[] {inputs.turnAbsolutePositionRad};
  }

  @Override
  public void setDriveVoltage(double volts) {
    m_driveAppliedVolts =
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE); // makes volts go from -12 to 12
    m_driveSim.setInputVoltage(m_driveAppliedVolts); // OMG Guess what this does
  }

  @Override
  public void setTurnVoltage(double volts) {
    m_turnAppliedVolts =
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE); // same as the drive voltage  
    m_turnSim.setInputVoltage(m_turnAppliedVolts); // tralalero tralala porcodilo porcola 
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    m_driveSetpointRadPerSec = velocityRadPerSec; // uy very hard to guess 
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    m_driveController.setPID(kP, kI, kD); // uy very hard to guess
  }

  @Override
  public void setDriveFF(double kS, double kV) {
    m_driveFeedforward.setKs(kS); // datebayo 
    m_driveFeedforward.setKv(kV);
  }
}
