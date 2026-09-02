package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.generated.TunerConstants;
import frc.robot.util.MapleSimSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

public class SwerveIOSim extends SwerveIOCTRE {

  private static final Frequency SIM_PERIOD_LOOP = Hertz.of(50);

  private final Notifier simThread;

  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
      moduleConstants;

  private final MapleSimSwerveDrivetrain mapleSimSwerve;

  public SwerveIOSim(
      SwerveDrivetrainConstants constants,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
          moduleConstants) {
    super(constants, moduleConstants);

    this.moduleConstants = moduleConstants;
    mapleSimSwerve = configureSimSwerve();

    simThread = new Notifier(mapleSimSwerve::update);
    simThread.startPeriodic(SIM_PERIOD_LOOP);
    
  }

  private MapleSimSwerveDrivetrain configureSimSwerve() {
    return new MapleSimSwerveDrivetrain(
        SIM_PERIOD_LOOP.asPeriod(),
        SwerveConstants.ROBOT_WEIGHT_WITH_BUMPERS,
        SwerveConstants.ROBOT_BUMPER_WIDTH,
        SwerveConstants.ROBOT_BUMPER_LENGTH,
        SwerveConstants.DRIVE_MOTOR_MODEL,
        SwerveConstants.STEER_MOTOR_MODEL,
        SwerveConstants.COF,
        getModuleLocations(),
        getPigeon2(),
        getModules(),
        moduleConstants);
  }

  @Override
  public void resetPose(Pose2d pose) {
    mapleSimSwerve.mapleSimDrive.setSimulationWorldPose(pose);
    Timer.delay(Seconds.of(0.05));
    super.resetPose(pose);
  }

  @Override
  public void updateInputs(
      SwerveIOInputs swerveInputs, GyroIOInputs gyroInputs, ModuleIOInputs... moduleInputs) {
    super.updateInputs(swerveInputs, gyroInputs, moduleInputs);

    Pose2d pose = super.getState().Pose;
    if (pose != null) {
      Logger.recordOutput(SwerveConstants.LOG_PATH + "Sim/Pose", pose);
    }
  }
}
