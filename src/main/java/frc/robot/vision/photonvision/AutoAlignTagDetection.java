package frc.robot.vision.photonvision;

import static frc.robot.constants.FieldConstants.aprilTags;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AutoAlignTagDetection extends SubsystemBase {
  private PhotonCamera camera;
  private Pose3d cameraPose;
  private PhotonPipelineResult latestResult = new PhotonPipelineResult();

  public AutoAlignTagDetection(PhotonCamera camera, Pose3d cameraPose) {
    this.camera = camera;
    this.cameraPose = cameraPose;
  }

  @Override
  public void periodic() {
    List<PhotonPipelineResult> unprocessedResults = camera.getAllUnreadResults();
    if (!unprocessedResults.isEmpty()) {
      latestResult = unprocessedResults.get(unprocessedResults.size() - 1);
    }

    if (latestResult.hasTargets()) {
      Logger.recordOutput("DriveToPose/tagDist", getRobotFrontDistanceToTag(18));
    }
  }

  public double getRobotFrontDistanceToTag(int targetID) {
    double camPitchDistanceToTag =
        PhotonUtils.calculateDistanceToTargetMeters(
            cameraPose.getZ(),
            aprilTags.getTagPose(targetID).get().getZ(),
            cameraPose.getRotation().getY(),
            Math.toRadians(getTag(targetID).pitch));

    double camDistanceToTag =
        camPitchDistanceToTag / Math.cos(Math.toRadians(getTag(targetID).yaw));
    Logger.recordOutput("DriveToPose/camDistToTag", camDistanceToTag);
    Translation2d cameraToTag =
        new Translation2d(camDistanceToTag, new Rotation2d(Math.toRadians(getTag(targetID).yaw)));
    // (drivebase length / 2 - camera X pos) + bumper offset gives offset front of robot
    Translation2d cameraToRobotFront =
        new Translation2d(
            (Units.inchesToMeters(13) - cameraPose.getX()) + Units.inchesToMeters(3.5), 0);
    return cameraToRobotFront.getDistance(cameraToTag);
  }

  public boolean hasTargetID(int targetID) {
    if (latestResult.hasTargets()) {
      for (PhotonTrackedTarget target : latestResult.getTargets()) {
        if (target.fiducialId == targetID) {
          return true;
        }
      }
    }
    return false;
  }

  public PhotonTrackedTarget getTag(int targetID) {
    if (latestResult.hasTargets()) {
      for (PhotonTrackedTarget target : latestResult.getTargets()) {
        if (target.fiducialId == targetID) {
          return target;
        }
      }
    }
    return new PhotonTrackedTarget();
  }
}
