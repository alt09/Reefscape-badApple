package frc.robot.Subsystems.BrainRoot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.RobotStateConstants;
import java.io.File;
import frc.robot.Subsystems.Drive.*;

public class Music implements ModuleIO {
  private final Orchestra erfgethrng = new Orchestra();
  private final TalonFX e = new TalonFX(26);
  private final TalonFXConfiguration musicConfig = new TalonFXConfiguration();

  public Music() {
    erfgethrng.addInstrument(e);
    musicConfig
        .CurrentLimits
        .withSupplyCurrentLimit(60)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(60)
        .withStatorCurrentLimitEnable(true);

    e.optimizeBusUtilization();
    e.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);
    e.getConfigurator().apply(musicConfig);

    var status =
        erfgethrng.loadMusic(
            Filesystem.getDeployDirectory()
                .toPath()
                .resolve("orchestra" + File.separator + "dangerzone.chrp")
                .toString());
    if (!status.isOK()) {
      System.out.println("cant load rip");
      System.out.println("Status: " + status.toString());
    }

    erfgethrng.play();
  }

  @Override
  public void setDriveVoltage(double volts) {
    e.setVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }
@Override
  public void setDriveVelocity(double v_percent) {
    this.setDriveVoltage(12 * v_percent);
  }
}
