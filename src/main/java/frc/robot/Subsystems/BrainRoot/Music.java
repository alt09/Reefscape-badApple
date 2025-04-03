package frc.robot.Subsystems.BrainRoot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.RobotStateConstants;
import java.io.File;

public class Music {
  private final Orchestra erfgethrng = new Orchestra();
  private final TalonFX e = new TalonFX(1, "music");
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
    e.setExpiration(100000000.00);
    e.getConfigurator().apply(musicConfig);

    var status =
        erfgethrng.loadMusic(
            Filesystem.getDeployDirectory()
                .toPath()
                .resolve("orchestra" + File.separator + "Never_gonna_give_you_up.mid")
                .toString());
    if (!status.isOK()) {
      System.out.println("cant load rip");
      System.out.println("Status: " + status.toString());
    }

    erfgethrng.play();
  }

  public void setVoltage(double volts) {
    e.setVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }

  public void setVelocity(double v_percent) {
    setVoltage(12 * v_percent);
  }
}
