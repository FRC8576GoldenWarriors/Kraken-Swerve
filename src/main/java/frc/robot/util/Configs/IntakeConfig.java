package frc.robot.util.Configs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class IntakeConfig {

    private int intakePivotId;
    private int intakeRollerId;

    private TalonFXConfiguration intakePivotConfiguration;
    private TalonFXConfiguration intakeRollerConfiguration;

    public IntakeConfig() {}

    public IntakeConfig withIntakePivotId(int id) {
        this.intakePivotId = id;
        return this;
    }

    public IntakeConfig withIntakeRollerId(int id) {
        this.intakeRollerId = id;
        return this;
    }

    public IntakeConfig withIntakePivotConfig(TalonFXConfiguration config) {
        this.intakePivotConfiguration = config;
        return this;
    }

    public IntakeConfig withIntakeRollerConfig(TalonFXConfiguration config) {
        this.intakeRollerConfiguration = config;
        return this;
    }

    public int getIntakePivotId() {
        return this.intakePivotId;
    }

    public int getIntakeRollerId() {
        return this.intakeRollerId;
    }

    public TalonFXConfiguration getIntakePivotConfiguration() {
        return this.intakePivotConfiguration;
    }

    public TalonFXConfiguration getIntakeRollerConfiguration() {
        return this.intakeRollerConfiguration;
    }

}