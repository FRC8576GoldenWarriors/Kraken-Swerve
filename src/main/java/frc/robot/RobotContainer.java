// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.WantedState;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveIOCTRE;
import frc.robot.subsystems.swerve.SwerveIOSim;

public class RobotContainer {
  private final CommandXboxController controller = new CommandXboxController(0);

  public final Swerve swerve;

  public RobotContainer() {
    swerve = buildSwerveSubsystem();
    configureBindings();
  }

  private void configureBindings() {

    swerve.setDefaultCommand(
        swerve
            .runEnd(
                () ->
                    swerve.acceptControllerInput(
                        controller.getLeftY(), controller.getLeftX(), controller.getRightX()),
                () -> swerve.acceptControllerInput(0, 0, 0))
            .withName("Accept Teleop Input"));

    controller.start().onTrue(Commands.runOnce(swerve::zeroHeading).withName("Reset Heading"));

    RobotModeTriggers.disabled()
        .onTrue(
            Commands.runOnce(() -> swerve.setWantedState(WantedState.IDLE))
                .ignoringDisable(true)
                .withName("Idle"));

    RobotModeTriggers.teleop()
        .and(
            () ->
                swerve.getWantedState() == WantedState.IDLE
                    || swerve.getWantedState() == WantedState.TELEOP)
        .onTrue(
            Commands.runOnce(() -> swerve.setWantedState(WantedState.TELEOP))
                .withName("Drive Teleop"));

    if (SwerveConstants.USE_SYS_ID_MODE) {
      controller
          .back()
          .onTrue(
              Commands.runOnce(() -> swerve.setWantedState(SwerveConstants.WANTED_SYS_ID_STATE)));
      controller.x().and(controller.a()).whileTrue(swerve.getDynamicForwardCommand());
      controller.a().and(controller.b()).whileTrue(swerve.getDynamicReverseCommand());
      controller.b().and(controller.y()).whileTrue(swerve.getQuasistaticForwardCommand());
      controller.y().and(controller.x()).whileTrue(swerve.getQuasistaticReverseCommand());
      controller
          .leftBumper()
          .whileTrue(
              swerve
                  .getWheelRadiusCharacterizationCommand()
                  .beforeStarting(
                      () -> swerve.setWantedState(WantedState.WHEEL_RADIUS_CHARACTERIZATION)));
      return;
    }

    controller
        .a()
        .onTrue(
            Commands.runOnce(() -> swerve.setWantedState(WantedState.ROTATION_LOCK))
                .beforeStarting(() -> swerve.setWantedRotation(new Rotation2d(Math.PI / 4)))
                .withName("Rotation Lock"))
        .onFalse(Commands.runOnce(() -> swerve.setWantedState(WantedState.IDLE)));

    controller
        .b()
        .onTrue(
            Commands.runOnce(() -> swerve.setWantedState(WantedState.WHEEL_LOCK_WITH_X))
                .withName("Wheel Lock With X"))
        .onFalse(Commands.runOnce(() -> swerve.setWantedState(WantedState.IDLE)));

    controller
        .x()
        .onTrue(
            Commands.runOnce(() -> swerve.setWantedState(WantedState.TAXI)).withName("Drive Taxi"))
        .onFalse(Commands.runOnce(() -> swerve.setWantedState(WantedState.IDLE)));

    // // Run SysId routines when holding back/start and X/Y.
    // // Note that each routine should be run exactly once in a single log.
    // controller.back().and(controller.y()).whileTrue(swerve.sysIdDynamic(Direction.kForward));
    // controller.back().and(controller.x()).whileTrue(swerve.sysIdDynamic(Direction.kReverse));
    // controller.start().and(controller.y()).whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
    // controller.start().and(controller.x()).whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));

  }

  public Command getAutonomousCommand() {
    /* Run the routine selected from the auto chooser */
    return null;
  }

  private Swerve buildSwerveSubsystem() {
    if (RobotBase.isReal()) {
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
          moduleConstants = new SwerveModuleConstants[4];

      moduleConstants[0] = TunerConstants.FrontLeft;
      moduleConstants[1] = TunerConstants.FrontRight;
      moduleConstants[2] = TunerConstants.BackLeft;
      moduleConstants[3] = TunerConstants.BackRight;

      return new Swerve(new SwerveIOCTRE(TunerConstants.DrivetrainConstants, moduleConstants));
    } else {
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
          moduleConstants = new SwerveModuleConstants[4];

      moduleConstants[0] = TunerConstants.FrontLeft;
      moduleConstants[1] = TunerConstants.FrontRight;
      moduleConstants[2] = TunerConstants.BackLeft;
      moduleConstants[3] = TunerConstants.BackRight;
      return new Swerve(new SwerveIOSim(TunerConstants.DrivetrainConstants, moduleConstants));
    }
  }
}
