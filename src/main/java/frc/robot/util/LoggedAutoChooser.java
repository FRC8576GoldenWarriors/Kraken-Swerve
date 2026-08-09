package frc.robot.util;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** A replacement for choreo's {@link choreo.auto.AutoChooser} that works with AdvantageKit. */
public class LoggedAutoChooser {
  private static final String NONE_NAME = "No Auto";

  private final LoggedDashboardChooser<String> impl;
  private final HashMap<String, Supplier<Command>> choices = new HashMap<>();
  private Optional<DriverStation.Alliance> alliance = Optional.empty();
  private Command selectedAuto = Commands.none();

  public LoggedAutoChooser(String name) {
    impl = new LoggedDashboardChooser<>(name);
    impl.onChange(this::onChange);
    new Trigger(() -> !DriverStation.getAlliance().equals(alliance))
        .and(DriverStation::isDisabled)
        .onTrue(
            Commands.runOnce(() -> onChange(impl.get()))
                .ignoringDisable(true)
                .withName("Auto Chooser Alliance Handler"));
    choices.put(NONE_NAME, Commands::none);
    impl.addDefaultOption(NONE_NAME, NONE_NAME);
  }

  private void onChange(String option) {
    alliance = DriverStation.getAlliance();
    selectedAuto = alliance.isEmpty() ? Commands.none() : choices.get(option).get();
  }

  /**
   * Adds a Command to the auto chooser.
   *
   * @see choreo.auto.AutoChooser#addCmd
   */
  public LoggedAutoChooser addCmd(String name, Supplier<Command> commandSupplier) {
    choices.put(name, commandSupplier);
    impl.addOption(name, name);
    return this;
  }

  /**
   * Add an AutoRoutine to the chooser.
   *
   * @see choreo.auto.AutoChooser#addRoutine
   */
  public LoggedAutoChooser addRoutine(String name, Supplier<AutoRoutine> routineSupplier) {
    return addCmd(name, () -> routineSupplier.get().cmd());
  }

  /**
   * Returns the currently selected command.
   *
   * @see choreo.auto.AutoChooser#selectedCommand
   */
  public Command selectedCommand() {
    return selectedAuto;
  }

  /**
   * Gets a Command that schedules the selected auto routine.
   *
   * @see AutoChooser#selectedCommandScheduler
   */
  public Command selectedCommandScheduler() {
    return Commands.deferredProxy(() -> selectedAuto).withName("Autonomous Scheduler");
  }
}
