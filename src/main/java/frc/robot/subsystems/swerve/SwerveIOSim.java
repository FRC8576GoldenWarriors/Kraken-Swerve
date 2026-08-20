package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.wpilibj.Notifier;

public class SwerveIOSim extends SwerveIOCTRE {

  private final Notifier simThread;

  private final SwerveModuleConstants<?, ?, ?>[] moduleConstants;

  public SwerveIOSim(
      SwerveDrivetrainConstants constants, SwerveModuleConstants<?, ?, ?>[] moduleConstants) {
    super(constants, moduleConstants);

    this.moduleConstants = moduleConstants;
    simThread = getSimThread();
  }

  private Notifier getSimThread() {
    return new Notifier(() -> {});
  }
}
