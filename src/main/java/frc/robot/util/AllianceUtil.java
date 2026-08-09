package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;

public class AllianceUtil {

  private static AllianceUtil allianceUtility;

  @AutoLogOutput(key = "AllianceUtil/AllianceColor")
  private AllianceColor allianceColor = AllianceColor.NONE;

  @SuppressWarnings("unused")
  private Trigger allianceColorAssigner;

  public enum AllianceColor {
    BLUE,
    RED,
    NONE
  }

  private AllianceUtil() {
    allianceColorAssigner =
        new Trigger(() -> DriverStation.getAlliance().isPresent())
            .onTrue(
                Commands.runOnce(
                        () -> {
                          DriverStation.getAlliance()
                              .ifPresent(
                                  alliance -> {
                                    allianceColor = convert(alliance);
                                  });
                        })
                    .ignoringDisable(true));
  }

  public static AllianceUtil getInstance() {
    if (allianceUtility == null) {
      allianceUtility = new AllianceUtil();
    }
    return allianceUtility;
  }

  public AllianceColor getAllianceColor() {
    return allianceColor;
  }

  public boolean isBlue() {
    return allianceColor == AllianceColor.BLUE;
  }

  public boolean isRed() {
    return allianceColor == AllianceColor.RED;
  }

  public boolean hasAlliance() {
    return allianceColor != AllianceColor.NONE;
  }

  public boolean shouldFlip() {
    return hasAlliance() && isRed();
  }

  private AllianceColor convert(Alliance alliance) {
    return switch (alliance) {
      case Red -> AllianceColor.RED;
      case Blue -> AllianceColor.BLUE;
    };
  }
}
