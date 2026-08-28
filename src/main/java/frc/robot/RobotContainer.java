// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.WantedState;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveIOCTRE;

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
          .rightBumper()
          .onTrue(
              Commands.sequence(
                  swerve.getAbsoluteModuleRotationsSettingCommand().withTimeout(Seconds.of(2)),
                  Commands.runOnce(
                      () -> swerve.setWantedState(SwerveConstants.WANTED_SYS_ID_STATE))));
      controller.x().whileTrue(swerve.getDynamicForwardCommand());
      controller.y().whileTrue(swerve.getDynamicReverseCommand());
      controller.a().whileTrue(swerve.getQuasistaticForwardCommand());
      controller.b().whileTrue(swerve.getQuasistaticReverseCommand());
      controller
          .leftBumper()
          .onTrue(
              Commands.sequence(
                  Commands.runOnce(
                      () -> swerve.setWantedState(WantedState.WHEEL_RADIUS_CHARACTERIZATION)),
                  swerve.getWheelRadiusCharacterizationCommand()));
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
  }

  public Command getAutonomousCommand() {
    /* Run the routine selected from the auto chooser */
    return null;
  }

  private Swerve buildSwerveSubsystem() {
    SwerveModuleConstants<?, ?, ?>[] moduleConstants = new SwerveModuleConstants[4];

    moduleConstants[0] = TunerConstants.FrontLeft;
    moduleConstants[1] = TunerConstants.FrontRight;
    moduleConstants[2] = TunerConstants.BackLeft;
    moduleConstants[3] = TunerConstants.BackRight;

    return new Swerve(new SwerveIOCTRE(TunerConstants.DrivetrainConstants, moduleConstants));
  }
}
