package frc.robot.subsystems.autos;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.AllianceUtil;
import frc.robot.util.LoggedAutoChooser;
import org.littletonrobotics.junction.Logger;

public class Autos {

  private final LoggedAutoChooser loggedAutoChooser;
  private final Swerve swerve;
  private final AutoFactory autoFactory;

  private final TrajectoryLogger<SwerveSample> trajectoryLogger;

  public Autos(Swerve swerve) {
    loggedAutoChooser = new LoggedAutoChooser(AutosConstants.LOGGED_AUTO_CHOOSER_KEY);
    this.swerve = swerve;
    this.trajectoryLogger =
        (trajectory, completionState) -> {
          Pose2d[] trajPoses =
              (AllianceUtil.getInstance().shouldFlip())
                  ? trajectory.flipped().getPoses()
                  : trajectory.getPoses();
          Logger.recordOutput(AutosConstants.CHOREO_LOG_PATH + "StartingPose", trajPoses[0]);
          Logger.recordOutput(AutosConstants.CHOREO_LOG_PATH + "Trajectory", trajPoses);
          Logger.recordOutput(
              AutosConstants.CHOREO_LOG_PATH + "TargetPose", trajPoses[trajPoses.length - 1]);
        };

    autoFactory =
        new AutoFactory(
            swerve::getPose,
            swerve::resetOdometry,
            swerve::setWantedSwerveSample,
            AutosConstants.USE_ALLIANCE_AUTOMATIC_FLIPPING,
            swerve,
            trajectoryLogger);

    CommandScheduler.getInstance().schedule(autoFactory.warmupCmd());
  }

  public Command getAutonomousCommand() {
    return loggedAutoChooser.selectedCommand();
  }
}
