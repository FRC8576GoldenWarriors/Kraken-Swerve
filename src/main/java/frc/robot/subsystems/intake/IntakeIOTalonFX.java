package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.util.Configs.IntakeConfig;

public class IntakeIOTalonFX implements IntakeIO {

    private final IntakeConfig intakeConfig;

    private final TalonFX intakePivotTalonFX;
    private final TalonFX intakeRollerTalonFX;

    private final MotionMagicVoltage intakePivotPositionRequest = new MotionMagicVoltage(Radians.zero());
    private final VelocityVoltage intakeRollerVelocityRequest = new VelocityVoltage(RadiansPerSecond.zero());

    public IntakeIOTalonFX(IntakeConfig config) {
        this.intakeConfig = config;
        
        this.intakePivotTalonFX = new TalonFX(intakeConfig.getIntakePivotId());
        this.intakeRollerTalonFX = new TalonFX(intakeConfig.getIntakeRollerId());

    }



}