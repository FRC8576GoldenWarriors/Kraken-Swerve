package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SetModuleRotationsAbsolute implements SwerveRequest {

  public Rotation2d[] moduleRotations = new Rotation2d[4];

  private static final SwerveModule.ModuleRequest moduleRequest = new SwerveModule.ModuleRequest();

  public SetModuleRotationsAbsolute withModuleRotations(final Rotation2d[] moduleRotations) {
    this.moduleRotations = moduleRotations;
    return this;
  }

  @Override
  public StatusCode apply(
      SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
    moduleRequest.withUpdatePeriod(parameters.updatePeriod);
    for (int i = 0; i < modulesToApply.length; i++) {
      final SwerveModule<?, ?, ?> module = modulesToApply[i];
      module.apply(moduleRequest.withState(new SwerveModuleState(0, moduleRotations[i])));
    }
    return StatusCode.OK;
  }
}
