package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotStateConstants;

public class RobotContainer {
  // Subsystems

  // Controllers
  private final CommandXboxController controller =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);

  public RobotContainer() {
    switch (RobotStateConstants.getMode()) {
      case REAL:
        break;
      case SIM:
        break;
      default:
        break;
    }
  }
}
