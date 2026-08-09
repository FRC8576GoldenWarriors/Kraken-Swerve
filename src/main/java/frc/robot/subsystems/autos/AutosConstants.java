package frc.robot.subsystems.autos;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.util.LoggedTunableNumber;

public class AutosConstants {
  public static final String LOGGED_AUTO_CHOOSER_KEY = "Auto Chooser";
  public static final boolean USE_ALLIANCE_AUTOMATIC_FLIPPING = false;

  public static final String CHOREO_LOG_PATH = "Autos/Choreo/";
  public static final String PATHPLANNER_LOG_PATH = "Autos/PathPlanner/";

  public static final String CHOREO_TUNING_PATH = "Choreo/";

  public static final LoggedTunableNumber CHOREO_X_LOGGED_KP =
      new LoggedTunableNumber(CHOREO_TUNING_PATH + "KP", 7, SwerveConstants.USE_TUNING_MODE);
  public static final LoggedTunableNumber CHOREO_X_LOGGED_KI =
      new LoggedTunableNumber(CHOREO_TUNING_PATH + "KI", 0, SwerveConstants.USE_TUNING_MODE);
  public static final LoggedTunableNumber CHOREO_X_LOGGED_KD =
      new LoggedTunableNumber(CHOREO_TUNING_PATH + "KD", 0, SwerveConstants.USE_TUNING_MODE);

  public static final LoggedTunableNumber CHOREO_Y_LOGGED_KP =
      new LoggedTunableNumber(CHOREO_TUNING_PATH + "KP", 7, SwerveConstants.USE_TUNING_MODE);
  public static final LoggedTunableNumber CHOREO_Y_LOGGED_KI =
      new LoggedTunableNumber(CHOREO_TUNING_PATH + "KI", 0, SwerveConstants.USE_TUNING_MODE);
  public static final LoggedTunableNumber CHOREO_Y_LOGGED_KD =
      new LoggedTunableNumber(CHOREO_TUNING_PATH + "KD", 0, SwerveConstants.USE_TUNING_MODE);

  public static final LoggedTunableNumber CHOREO_THETA_LOGGED_KP =
      new LoggedTunableNumber(CHOREO_TUNING_PATH + "KP", 7, SwerveConstants.USE_TUNING_MODE);
  public static final LoggedTunableNumber CHOREO_THETA_LOGGED_KI =
      new LoggedTunableNumber(CHOREO_TUNING_PATH + "KI", 0, SwerveConstants.USE_TUNING_MODE);
  public static final LoggedTunableNumber CHOREO_THETA_LOGGED_KD =
      new LoggedTunableNumber(CHOREO_TUNING_PATH + "KD", 0, SwerveConstants.USE_TUNING_MODE);
  public static final Constraints CHOREO_THETA_CONSTRAINTS =
      new Constraints(
          SwerveConstants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond),
          SwerveConstants.MAX_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond));
}
