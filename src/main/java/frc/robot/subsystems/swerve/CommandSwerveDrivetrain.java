package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import frc.robot.Robot;
import frc.robot.Robot.Mode;
import frc.robot.util.MapleSimSwerveDrivetrain;

public class CommandSwerveDrivetrain {
  SwerveDrivetrainConstants driveTrainConstants;
  SwerveModuleConstants<?, ?, ?>[] moduleConstants;

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    this.driveTrainConstants = driveTrainConstants;
    // Regulate module constants if in simulation mode
    if (Robot.mode == Mode.SIM) {
      this.moduleConstants = MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules);
    } else {
      this.moduleConstants = modules;
    }
    this.moduleConstants = modules;
  }

  public SwerveDrivetrainConstants getDriveTrainConstants() {
    return driveTrainConstants;
  }

  public SwerveModuleConstants<?, ?, ?>[] getModuleConstants() {
    return moduleConstants;
  }
}
