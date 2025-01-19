package frc.robot.vision.photonvision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.constants.FieldConstants.aprilTags;

import java.util.List;
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
    latestResult = unprocessedResults.get(unprocessedResults.size() - 1);
  }

  public double getRobotFrontDistanceToTag(int targetID) {
    double camDistanceToTag = PhotonUtils.calculateDistanceToTargetMeters(cameraPose.getZ(), aprilTags.getTagPose(targetID).get().getZ(), cameraPose.getRotation().getY(), getTag(targetID).pitch);
    Translation2d cameraToTag = new Translation2d(camDistanceToTag, new Rotation2d(Math.toRadians(getTag(targetID).yaw)));
    // (drivebase length / 2 - camera X pos) + bumper offset gives offset front of robot
    Translation2d cameraToRobotFront = new Translation2d((13-cameraPose.getX()) + 3.5,0);
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