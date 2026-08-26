package frc.robot.util;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SimSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.swerve.SwerveConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class MapleSimSwerve {
  private final Pigeon2SimState pigeon2SimState;
  private final SimSwerveModule[] simSwerveModules;
  private final SimSwerveDrivetrain simSwerveDrivetrain;

  public MapleSimSwerve() {
    pigeon2SimState = null;
    simSwerveModules = null;
    simSwerveDrivetrain = null;
  }

  public void update() {
    SimulatedArena.getInstance().simulationPeriodic();
    pigeon2SimState.setRawYaw(null);
    pigeon2SimState.setAngularVelocityZ(null);
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
}
