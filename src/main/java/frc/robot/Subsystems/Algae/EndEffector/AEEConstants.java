package frc.robot.Subsystems.Algae.EndEffector;

import edu.wpi.first.math.util.Units;

public class AEEConstants {
  // REAL CONSTANTS
  /** CAN ID for the AEE SPARK MAX */
  public static final int CAN_ID = 21;
  /** Gear reduction of 6:1 for the AEE motor */
  public static final double GEAR_RATIO = 6.0 / 1.0;
  /**
   * Set the inversion status of the AEE to false, making Counterclockwise the positive direction
   */
  public static final boolean IS_INVERTED = false;
  /** Current limit for the NEO motor of the AEE */
  public static final int CUR_LIM_A = 20;
  // Velocities
  /** Scoring velocity in radians per second */
  public static final double SCORE_VELOCITY_RAD_PER_SEC =
      Units.rotationsPerMinuteToRadiansPerSecond(-1000);
  /** Intaking velocity in radians per second */
  public static final double INTAKE_VELOCITY_RAD_PER_SEC =
      Units.rotationsPerMinuteToRadiansPerSecond(1000);
  /** Scoring speed, open loop voltage control */
  public static final double SCORE_PERCENT_SPEED = 0.80;
  /** Intaking speed, open loop voltage control */
  public static final double INTAKE_PERCENT_SPEED = -0.80;
  /** Speed when removing ALGAE from the REEF, open loop voltage control */
  public static final double DEALGAE_PERCENT_SPEED = 0.90;
  /** Speed when holding an ALGAE in the AEE */
  public static final double ALGAE_HOLD_PERCENT_SPEED = -0.20;

  // PID CONSTANTS
  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static double KP = 1.0;
  /**
   * KI represents the constant multiplied by the integral of the error from setpoint (Integral
   * Error)
   */
  public static double KI = 0.0;
  /** KD represents the constant multiplied by the change in error over time (Derivative Error) */
  public static double KD = 0.0;

  // SIM CONSTANTS
  /** Moment of inertia for the AEE flywheels in kilograms * meters squared */
  public static final double MOI_KG_M2 = 0.0025;
}
