package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.RobotStateConstants;
import java.io.File;
import java.util.Queue;

/** ModuleIO implementation for the real mode of the robot */
public class ModuleIOSparkMaxTalonFX implements ModuleIO {
  // Drive motor, controller, and configurator
  private final TalonFX m_driveTalonFX; // creates a talon fx aka a krakenX60
  private final VelocityVoltage m_driveController = new VelocityVoltage(0); // creates a velocity controller for the krakenX60
  private final TalonFXConfiguration m_driveConfig = new TalonFXConfiguration(); //  creates the configuration for the krakenX60

  // Turn motor, absolute encoder, controller, and configurator
  private final SparkMax m_turnSparkMax; // creates a spark max aka a neo or neo 550 or any motor that is conected to a spark max
  private final SparkMaxConfig m_turnConfig = new SparkMaxConfig(); // creates the configuration for the spark max not the motor
  private final CANcoder m_turnCANcoder; // creates a can coder aka a absolute encoder
  private final double m_absEncoderOffsetRad; // the offset of the absolute encoder in radians

  // Drive motor's logged signals
  private final StatusSignal<Voltage> m_driveAppliedVolts; 
  private final StatusSignal<Current> m_driveCurrentAmps;
  private final StatusSignal<Temperature> m_driveTempCelsius; // we don't need this but ok 
  private final StatusSignal<Angle> m_drivePositionRot; // Rotations
  private final StatusSignal<AngularVelocity> m_driveVelocityRotPerSec; // Rotations per second

  // CANcoder's logged signals
  private final StatusSignal<Angle> m_absEncoderPositionRot; // Rotations
  private final StatusSignal<AngularVelocity> m_absEncoderVelocityRotPerSec; // Rotations per second

  // PhoenixOdometryThread queues
  private final Queue<Double> m_timestampQueue; // bruh this time IDK what this does 
  private final Queue<Double> m_drivePositionQueue; // your cooked 
  private final Queue<Double> m_absEncoderPositionQueue; // RIP
  private final Orchestra m_orchestra = new Orchestra();// this is completely necessary, follow this if you wanna be a pro

  /**
   * Constructs a new {@link ModuleIOSparkMaxTalonFX} instance.
   *
   * <p>This creates a new {@link ModuleIO} object that uses the real KrakenX60 and NEO motors to
   * run the Drive and Turn of the Module and the CANcoder for the absolute Turn position of the
   * wheels.
   *
   * @param moduleNumber Number to the corresponding Swerve Module that is to be initilized.
   */
  public ModuleIOSparkMaxTalonFX(int moduleNumber) {
    System.out.println("[Init] Creating ModuleIOSparkMaxTalonFX " + moduleNumber); //for debugging

    // Initialize Drive motors, Turn motors, Turn encoders and their offsets based on the Module
    // number
    switch (moduleNumber) { // for each module do this
      case 0:
        m_driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.FRONT_LEFT.CAN_ID, "Drivetrain"); // drive is a krakenX60 with a can id and a canbus
        m_turnSparkMax =
            new SparkMax(DriveConstants.TURN_MOTOR.FRONT_LEFT.CAN_ID, MotorType.kBrushless); //turn is a neo aka brushhless and a can id 
        m_turnCANcoder =
            new CANcoder(DriveConstants.ABSOLUTE_ENCODER.FRONT_LEFT.CAN_ID, "Drivetrain"); // absolute encoder with a can id and a canbus
        m_absEncoderOffsetRad = DriveConstants.ABSOLUTE_ENCODER_OFFSET.FRONT_LEFT.OFFSET; // offset for each encoder
        m_orchestra.addInstrument(m_driveTalonFX); // add this to the orchestra TODO:important don't msiss this 
        break;

      case 1:
        m_driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.FRONT_RIGHT.CAN_ID, "Drivetrain"); // drive is a krakenX60 with a can id and a canbus
        m_turnSparkMax =
            new SparkMax(DriveConstants.TURN_MOTOR.FRONT_RIGHT.CAN_ID, MotorType.kBrushless); //turn is a neo aka brushhless and a can id
        m_turnCANcoder =
            new CANcoder(DriveConstants.ABSOLUTE_ENCODER.FRONT_RIGHT.CAN_ID, "Drivetrain"); // absolute encoder with a can id and a canbus
        m_absEncoderOffsetRad = DriveConstants.ABSOLUTE_ENCODER_OFFSET.FRONT_RIGHT.OFFSET;  // offset for each encoder
        m_orchestra.addInstrument(m_driveTalonFX); // add this to the orchestra TODO:important don't msiss this
        break;

      case 2:
        m_driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.BACK_LEFT.CAN_ID, "Drivetrain"); // drive is a krakenX60 with a can id and a canbus
        m_turnSparkMax =
            new SparkMax(DriveConstants.TURN_MOTOR.BACK_LEFT.CAN_ID, MotorType.kBrushless); //turn is a neo aka brushhless and a can id
        m_turnCANcoder =
            new CANcoder(DriveConstants.ABSOLUTE_ENCODER.BACK_LEFT.CAN_ID, "Drivetrain"); // absolute encoder with a can id and a canbus
        m_absEncoderOffsetRad = DriveConstants.ABSOLUTE_ENCODER_OFFSET.BACK_LEFT.OFFSET; // offset for each encoder
        m_orchestra.addInstrument(m_driveTalonFX); // add this to the orchestra TODO:important don't msiss this
        break;

      case 3:
        m_driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.BACK_RIGHT.CAN_ID, "Drivetrain"); // drive is a krakenX60 with a can id and a canbus
        m_turnSparkMax =
            new SparkMax(DriveConstants.TURN_MOTOR.BACK_RIGHT.CAN_ID, MotorType.kBrushless); //turn is a neo aka brushhless and a can id
        m_turnCANcoder =
            new CANcoder(DriveConstants.ABSOLUTE_ENCODER.BACK_RIGHT.CAN_ID, "Drivetrain"); // absolute encoder with a can id and a canbus
        m_absEncoderOffsetRad = DriveConstants.ABSOLUTE_ENCODER_OFFSET.BACK_RIGHT.OFFSET; // offset for each encoder
        m_orchestra.addInstrument(m_driveTalonFX); // add this to the orchestra TODO:important don't msiss this
        break;

      default:
        throw new RuntimeException("Invalid Module number for ModuleIOSparkMaxTalonFX"); // if the module number is invalid then throw this
    }

    // TalonFX motor configurations
    m_driveConfig
        .MotorOutput
        .withInverted(
            DriveConstants.DRIVE_IS_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive) // if is inverted the drive, you normllly don't want this
        .withNeutralMode(NeutralModeValue.Brake) // be brake mode, to don't move without voltage the motor 
        .withControlTimesyncFreqHz(DriveConstants.ODOMETRY_UPDATE_FREQUENCY_HZ); // update every 0.02 sec 

    // TalonFX current limit configurations
    m_driveConfig
        .CurrentLimits
        .withSupplyCurrentLimit(DriveConstants.CUR_LIM_A) // 60 amps is the max they can handle 
        .withSupplyCurrentLimitEnable(DriveConstants.ENABLE_CUR_LIM) // always enable it 
        .withStatorCurrentLimit(DriveConstants.CUR_LIM_A) // 60 amps is the max they can handle
        .withStatorCurrentLimitEnable(DriveConstants.ENABLE_CUR_LIM); // always enable it

    // TalonFX PID and Feedforward gains configuration
    m_driveConfig
        .Slot0
        .withKP(DriveConstants.DRIVE_KP)
        .withKI(DriveConstants.DRIVE_KI)
        .withKD(DriveConstants.DRIVE_KD)
        .withKS(DriveConstants.DRIVE_KS)
        .withKV(DriveConstants.DRIVE_KV); // bruh GL your cooked

    // TalonFX closed loop configurations
    m_driveConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(
        1.0 / DriveConstants.ODOMETRY_UPDATE_FREQUENCY_HZ); // update every 0.02 sec be careful
    m_driveController.withUpdateFreqHz(DriveConstants.ODOMETRY_UPDATE_FREQUENCY_HZ); // update every 0.02 sec be careful

    // Initialize Drive encoder position
    m_driveTalonFX.setPosition(0.0); // set the position to 0 to reset the pose 

    // SPARK MAX configurations
    m_turnConfig
        .inverted(DriveConstants.TURN_IS_INVERTED) // if is inverted the turn, you normllly don't want this
        .idleMode(IdleMode.kBrake) // be brake mode, to don't move without voltage the motor
        .smartCurrentLimit(DriveConstants.CUR_LIM_A); // 60 amps is the max they can handle

    // Optimize CAN bus usage, disable all signals besides those refreshed in code
    m_driveTalonFX.optimizeBusUtilization();
    m_turnCANcoder.optimizeBusUtilization(); // always optimeize the bus utilization

    // Set CAN timeouts
    m_driveTalonFX.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC); // TODO : if you habe an issue is probably this 
    m_turnSparkMax.setCANTimeout(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC * 1000); // same thing 

    // Apply TalonFX configurations
    m_driveTalonFX.getConfigurator().apply(m_driveConfig); // apply the confuguration

    // Apply SPARK MAX configurations
    m_turnSparkMax.configure(
        m_turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  // apply the confuguration

    // Initialize logged Drive motor signals
    m_drivePositionRot = m_driveTalonFX.getPosition();
    m_driveVelocityRotPerSec = m_driveTalonFX.getVelocity();
    m_driveAppliedVolts = m_driveTalonFX.getMotorVoltage();
    m_driveCurrentAmps = m_driveTalonFX.getStatorCurrent();
    m_driveTempCelsius = m_driveTalonFX.getDeviceTemp();

    // Initialize logged Absolute Encoder signals
    m_absEncoderPositionRot = m_turnCANcoder.getAbsolutePosition();
    m_absEncoderVelocityRotPerSec = m_turnCANcoder.getVelocity();

    // Initialize odometry queue singals
    m_timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    m_drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(m_driveTalonFX.getPosition());
    m_absEncoderPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(m_turnCANcoder.getAbsolutePosition());

    // Set update frequencies for signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        DriveConstants.UPDATE_FREQUENCY_HZ,
        m_driveVelocityRotPerSec,
        m_driveAppliedVolts,
        m_driveCurrentAmps,
        m_driveTempCelsius,
        m_absEncoderVelocityRotPerSec);
    // Use higher frequency (250 Hz) for position signals or more accurate odometry
    BaseStatusSignal.setUpdateFrequencyForAll(
        DriveConstants.ODOMETRY_UPDATE_FREQUENCY_HZ, m_drivePositionRot, m_absEncoderPositionRot);

    var status = // TODO:important don't miss this
        m_orchestra.loadMusic(
            Filesystem.getDeployDirectory()
                .toPath()
                .resolve("orchestra" + File.separator + ) // copy the same files that are here on src/main/deploy/orchestra/
                .toString());

    if (!status.isOK()) {
      System.out.println(
          "[Error] Failed to load orchestra music file: dangerzone.chrp. Status: "
              + status.toString());
    }
    m_orchestra.play();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update Drive motor signals and check if they are recieved
    inputs.driveIsConnected =
        BaseStatusSignal.refreshAll(
                m_driveAppliedVolts,
                m_driveCurrentAmps,
                m_driveTempCelsius,
                m_driveVelocityRotPerSec,
                m_drivePositionRot)
            .isOK();
    // Update Drive motor logged inputs
    inputs.driveAppliedVoltage = m_driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = m_driveCurrentAmps.getValueAsDouble();
    inputs.driveTempCelsius = m_driveTempCelsius.getValueAsDouble();
    inputs.drivePositionRad =
        Units.rotationsToRadians(m_drivePositionRot.getValueAsDouble())
            / DriveConstants.DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_driveVelocityRotPerSec.getValueAsDouble() * 60)
            / DriveConstants.DRIVE_GEAR_RATIO;

    // Update Absolute Encoder signals and check if they are recieved
    inputs.absoluteEncoderIsConnected =
        BaseStatusSignal.refreshAll(m_absEncoderPositionRot, m_absEncoderVelocityRotPerSec).isOK();
    // Update Turn motor and Absolute Encoder logged inputs
    inputs.turnAppliedVoltage = m_turnSparkMax.getAppliedOutput() * m_turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = m_turnSparkMax.getOutputCurrent();
    inputs.turnTempCelsius = m_turnSparkMax.getMotorTemperature();
    inputs.turnAbsolutePositionRad =
        Rotation2d.fromRadians(
            MathUtil.angleModulus(
                Units.rotationsToRadians(m_absEncoderPositionRot.getValueAsDouble())
                    + m_absEncoderOffsetRad));
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(m_absEncoderVelocityRotPerSec.getValueAsDouble());

    // Update odometry queues
    inputs.odometryTimestamps =
        m_timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        m_drivePositionQueue.stream()
            .mapToDouble(
                (Double value) -> Units.rotationsToRadians(value / DriveConstants.DRIVE_GEAR_RATIO))
            .toArray();
    inputs.odometryAbsTurnPositions =
        m_absEncoderPositionQueue.stream()
            .map(
                (Double value) ->
                    Rotation2d.fromRadians(
                        MathUtil.angleModulus(
                            Units.rotationsToRadians(value) + m_absEncoderOffsetRad)))
            .toArray(Rotation2d[]::new);
    // Clear queues for a new set of signals next robot cycle
    m_timestampQueue.clear();
    m_drivePositionQueue.clear();
    m_absEncoderPositionQueue.clear();
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    m_driveTalonFX.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    m_turnConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    m_turnSparkMax.configure(
        m_turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setDriveVoltage(double volts) {
    m_driveTalonFX.setVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }

  @Override
  public void setTurnVoltage(double volts) {
    m_turnSparkMax.setVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }

  /**
   * Sets the velocity of the Drive motor using the closed loop controller built into the TalonFX
   * speed controller.
   *
   * @param velocityRadPerSec Velocity to set Drive motor to in radians per second.
   */
  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    m_driveTalonFX.setControl(m_driveController.withVelocity(velocityRadPerSec));
  }

  /**
   * Sets the PID gains for the Drive motor's built in closed loop controller.
   *
   * @param kP Proportional gain value.
   * @param kI Integral gain value.
   * @param kD Derivative gain value.
   */
  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    // Configure new gains
    m_driveConfig.Slot0.withKP(kP).withKI(kI).withKD(kD);
    // Apply configuration
    m_driveTalonFX.getConfigurator().apply(m_driveConfig);
  }

  /**
   * Sets the Feedforward gains for the Drive motor's built in closed loop controller.
   *
   * @param kS Static gain value.
   * @param kV Velocity gain value.
   */
  @Override
  public void setDriveFF(double kS, double kV) {
    // Configure new gains
    m_driveConfig.Slot0.withKS(kS).withKV(kV);
    // Apply configuration
    m_driveTalonFX.getConfigurator().apply(m_driveConfig);
  }
}
