package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.swerve.SwerveConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class MapleSimSwerve {
  private final Pigeon2SimState pigeon2SimState;
  private final SimSwerveModule[] simSwerveModules;
  private final SwerveDriveSimulation simSwerveDrivetrain;

  public MapleSimSwerve(
      Frequency simulationFrequency,
      Mass robotMassWithBumpers,
      Distance bumperLengthX,
      Distance bumperLengthY,
      DCMotor driveMotorModel,
      DCMotor steerMotorModel,
      double wheelCOF,
      Translation2d[] moduleLocations,
      Pigeon2 pigeon,
      SwerveModule<TalonFX, TalonFX, CANcoder>[] modules,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
          moduleConstants) {
    pigeon2SimState = pigeon.getSimState();
    simSwerveModules = new SimSwerveModule[moduleConstants.length];
    DriveTrainSimulationConfig simConfig =
        DriveTrainSimulationConfig.Default()
            .withRobotMass(robotMassWithBumpers)
            .withBumperSize(bumperLengthX, bumperLengthY)
            .withCustomModuleTranslations(moduleLocations)
            .withSwerveModule(
                new SwerveModuleSimulationConfig(
                    driveMotorModel,
                    steerMotorModel,
                    moduleConstants[0].DriveMotorGearRatio,
                    moduleConstants[0].SteerMotorGearRatio,
                    Volts.of(moduleConstants[0].DriveFrictionVoltage),
                    Volts.of(moduleConstants[0].SteerFrictionVoltage),
                    Meters.of(moduleConstants[0].WheelRadius),
                    KilogramSquareMeters.of(moduleConstants[0].SteerInertia),
                    wheelCOF));
    simSwerveDrivetrain = new SwerveDriveSimulation(simConfig, Pose2d.kZero);

    SwerveModuleSimulation[] moduleSimulations = simSwerveDrivetrain.getModules();

    for (int i = 0; i < this.simSwerveModules.length; i++) {
      simSwerveModules[i] =
          new SimSwerveModule(moduleConstants[i], moduleSimulations[i], modules[i]);
    }

    SimulatedArena.overrideSimulationTimings(simulationFrequency.asPeriod(), 1);
    SimulatedArena.getInstance().addDriveTrainSimulation(simSwerveDrivetrain);
  }

  public void update() {
    SimulatedArena.getInstance().simulationPeriodic();
    pigeon2SimState.setRawYaw(
        simSwerveDrivetrain.getSimulatedDriveTrainPose().getRotation().getMeasure());
    pigeon2SimState.setAngularVelocityZ(
        RadiansPerSecond.of(
            simSwerveDrivetrain.getDriveTrainSimulatedChassisSpeedsRobotRelative()
                .omegaRadiansPerSecond));
  }

  private static final class SimSwerveModule {
    private final SwerveModuleConstants<?, ?, ?> swerveModuleConstant;
    private final SwerveModuleSimulation simSwerveModule;

    public SimSwerveModule(
        SwerveModuleConstants<?, ?, ?> swerveModuleConstant,
        SwerveModuleSimulation simSwerveModule,
        SwerveModule<TalonFX, TalonFX, CANcoder> swerveModule) {
      this.swerveModuleConstant = swerveModuleConstant;
      this.simSwerveModule = simSwerveModule;
      simSwerveModule.useDriveMotorController(
          new TalonFXMotorControllerSim(swerveModule.getDriveMotor()));
      simSwerveModule.useSteerMotorController(
          new TalonFXMotorControllerRemoteCANcoderSim(
              swerveModule.getDriveMotor(), swerveModule.getEncoder()));
    }
  }

  private static class TalonFXMotorControllerSim implements SimulatedMotorController {
    private final int id;

    private final TalonFXSimState talonFXSimState;

    public TalonFXMotorControllerSim(TalonFX talonFX) {
      this.id = talonFX.getDeviceID();
      this.talonFXSimState = talonFX.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      talonFXSimState.setRawRotorPosition(encoderAngle);
      talonFXSimState.setRotorVelocity(encoderVelocity);
      talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

      return talonFXSimState.getMotorVoltageMeasure();
    }
  }

  private static class TalonFXMotorControllerRemoteCANcoderSim extends TalonFXMotorControllerSim {
    private final CANcoderSimState remoteCanCoder;

    public TalonFXMotorControllerRemoteCANcoderSim(TalonFX talonFX, CANcoder remoteCANCoder) {
      super(talonFX);
      this.remoteCanCoder = remoteCANCoder.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      remoteCanCoder.setRawPosition(mechanismAngle);
      remoteCanCoder.setVelocity(mechanismVelocity);
      remoteCanCoder.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

      return super.updateControlSignal(
          mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
    }
  }

  public static SwerveModuleConstants<?, ?, ?>[] regulateModuleConstantsForSimulation(
      SwerveModuleConstants<?, ?, ?>[] swerveModuleConstants) {
    for (SwerveModuleConstants<?, ?, ?> swerveModuleConstant : swerveModuleConstants) {
      regulateModuleConstantForSimulation(swerveModuleConstant);
    }
    return swerveModuleConstants;
  }

  private static void regulateModuleConstantForSimulation(
      SwerveModuleConstants<?, ?, ?> swerveModuleConstant) {

    if (RobotBase.isReal()) return;
    var module = TunerConstants.FrontLeft;

    swerveModuleConstant
        .withEncoderOffset(0)
        .withDriveMotorInverted(false)
        .withSteerMotorInverted(false)
        .withEncoderInverted(false)
        .withDriveFrictionVoltage(module.DriveFrictionVoltage)
        .withSteerFrictionVoltage(module.SteerFrictionVoltage)
        .withDriveInertia(module.DriveInertia)
        .withSteerInertia(module.SteerInertia)
        .withDriveMotorGains(
            new Slot0Configs()
                .withKP(SwerveConstants.SIM_DRIVE_KP)
                .withKI(SwerveConstants.SIM_DRIVE_KI)
                .withKD(SwerveConstants.SIM_DRIVE_KD)
                .withKS(SwerveConstants.SIM_DRIVE_KS)
                .withKV(SwerveConstants.SIM_DRIVE_KV)
                .withKA(SwerveConstants.SIM_DRIVE_KA))
        .withSteerMotorGains(
            new Slot0Configs()
                .withKP(SwerveConstants.SIM_STEER_KP)
                .withKI(SwerveConstants.SIM_STEER_KI)
                .withKD(SwerveConstants.SIM_STEER_KD)
                .withKS(SwerveConstants.SIM_STEER_KS)
                .withKV(SwerveConstants.SIM_STEER_KV)
                .withKA(SwerveConstants.SIM_STEER_KA)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign));
  }

  public SwerveDriveSimulation getMapleSwerveDrivetrainSimulation() {
    return simSwerveDrivetrain;
  }
}
