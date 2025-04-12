package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Subsystems.Algae.EndEffector.*;
import frc.robot.Subsystems.Algae.Pivot.*;
import frc.robot.Subsystems.BrainRoot.Music;
import frc.robot.Subsystems.Climber.*;
import frc.robot.Subsystems.CoralEndEffector.*;
import frc.robot.Subsystems.Drive.*;
import frc.robot.Subsystems.Funnel.*;
import frc.robot.Subsystems.Periscope.*;
import frc.robot.Subsystems.Vision.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  // Chassis
  //   private final Drive m_driveSubsystem;

  //   // Mechanisms
  //   private final AlgaePivot m_algaePivotSubsystem;
  //   private final Periscope m_periscopeSubsystem;
  //   private final Climber m_climberSubsystem;
  //   private final Funnel m_funnelSubsystem;

  private final Music music;

  //   private final AEE m_AEESubsystem;
  //   private final CEE m_CEESubsystem;

  // Utils
  //   private final Vision m_visionSubsystem;

  // Controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);
  private final CommandXboxController m_auxButtonBoard =
      new CommandXboxController(OperatorConstants.AUX_BUTTON_BOARD);
  private final CommandXboxController m_auxController =
      new CommandXboxController(OperatorConstants.AUX_XBOX_CONTROLLER);

  // Autos
  private final LoggedDashboardChooser<Command> m_autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    switch (RobotStateConstants.getMode()) {
        // Real robot, instantiates hardware IO implementations
      case REAL:
        // m_driveSubsystem =
        //     new Drive(
        //         new ModuleIOSparkMaxTalonFX(0),
        //         new ModuleIOSparkMaxTalonFX(1),
        //         new ModuleIOSparkMaxTalonFX(2),
        //         new ModuleIOSparkMaxTalonFX(3),
        //         new GyroIOPigeon2());
        // m_algaePivotSubsystem = new AlgaePivot(new AlgaePivotIOSparkMax());
        // m_periscopeSubsystem = new Periscope(new PeriscopeIOTalonFX());
        // m_climberSubsystem = new Climber(new ClimberIOTalonFX());
        // m_funnelSubsystem = new Funnel(new FunnelIOSparkMax());
        // m_AEESubsystem = new AEE(new AEEIOSparkMax() {});
        // m_CEESubsystem = new CEE(new CEEIOSparkMax());
        // m_visionSubsystem =
        //     new Vision(
        //         m_driveSubsystem::addVisionMeasurement,
        //         new VisionIOPhotonVision(VisionConstants.CAMERA.FRONT.CAMERA_INDEX),
        //         new VisionIOPhotonVision(VisionConstants.CAMERA.BACK.CAMERA_INDEX));
        music = new Music();
        break;
        // Sim robot, instantiates physics sim IO implementations
      case SIM:
        // m_driveSubsystem =
        //     new Drive(
        //         new ModuleIOSim(),
        //         new ModuleIOSim(),
        //         new ModuleIOSim(),
        //         new ModuleIOSim(),
        //         new GyroIO() {});
        // m_algaePivotSubsystem = new AlgaePivot(new AlgaePivotIOSim());
        // m_periscopeSubsystem = new Periscope(new PeriscopeIOSim());
        // m_climberSubsystem = new Climber(new ClimberIOSim());
        // m_funnelSubsystem = new Funnel(new FunnelIOSim());
        // m_AEESubsystem = new AEE(new AEEIOSim() {});
        // m_CEESubsystem = new CEE(new CEEIOSim());
        // m_visionSubsystem =
        //     new Vision(
        //         m_driveSubsystem::addVisionMeasurement,
        //         // new VisionIOSim(
        //         //     VisionConstants.CAMERA.FRONT.CAMERA_INDEX,
        //         // m_driveSubsystem::getCurrentPose2d),
        //         // new VisionIOSim(
        //         //     VisionConstants.CAMERA.BACK.CAMERA_INDEX,
        //         // m_driveSubsystem::getCurrentPose2d))
        //         new VisionIO() {});
        music = null;
        break;
        // Replayed robot, disables all IO implementations
      default:
        // m_driveSubsystem =
        //     new Drive(
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         new GyroIO() {});
        // m_algaePivotSubsystem = new AlgaePivot(new AlgaePivotIO() {});
        // m_periscopeSubsystem = new Periscope(new PeriscopeIO() {});
        // m_climberSubsystem = new Climber(new ClimberIO() {});
        // m_funnelSubsystem = new Funnel(new FunnelIO() {});
        // m_AEESubsystem = new AEE(new AEEIO() {});
        // m_CEESubsystem = new CEE(new CEEIO() {});
        music = null;
        // m_visionSubsystem = new Vision(m_driveSubsystem::addVisionMeasurement, new VisionIO()
        // {});
        break;
    }

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    this.driverControllerBindings();
    this.auxButtonBoardBindings();
    this.auxControllerBindings();
  }

  /** Driver Controls */
  private void driverControllerBindings() {
    m_driverController.a().onTrue(new InstantCommand(() -> music.setVelocity(0.8)));
  }

  /** Aux Button Board Controls */
  public void auxButtonBoardBindings() {}

  /** Aux Xbox Controls */
  public void auxControllerBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }

  /**
   * Sets all mechanisms to brake mode, intended for use when the robot is disabled.
   *
   * @param enable {@code true} to enable brake mode, {@code false} to enable coast mode.
   */
  public void allMechanismsBrakeMode(boolean enable) {}
}
