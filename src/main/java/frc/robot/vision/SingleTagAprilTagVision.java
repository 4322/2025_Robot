package frc.robot.vision;

import static frc.robot.constants.FieldConstants.aprilTagFieldLayout;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commons.GeomUtil;
import frc.robot.commons.PolynomialRegression;
import frc.robot.commons.TimestampedVisionUpdate;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class SingleTagAprilTagVision extends SubsystemBase {

  private PhotonCamera frontLeftCamera;
  private PhotonCamera frontRightCamera;

  private static final double fieldBorderMargin = 0.5;
  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = (x) -> {};
  private List<TimestampedVisionUpdate> visionUpdates;
  private Supplier<Pose2d> poseSupplier = () -> new Pose2d();
  private PhotonPipelineResult latestResult = new PhotonPipelineResult();
  private int targetTagID;

  private PolynomialRegression xyStdDevModel =
      new PolynomialRegression(
          new double[] {
            0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
            3.223358, 4.093358, 4.726358
          },
          new double[] {0.005, 0.0135, 0.016, 0.038, 0.0515, 0.0925, 0.12, 0.14, 0.17, 0.27, 0.38},
          2);

  public SingleTagAprilTagVision(PhotonCamera frontLeftCamera, PhotonCamera frontRightCamera) {
    this.frontLeftCamera = frontLeftCamera;
    this.frontRightCamera = frontRightCamera;
  }

  public void setDataInterfaces(
      Supplier<Pose2d> poseSupplier, Consumer<List<TimestampedVisionUpdate>> visionConsumer) {
    this.poseSupplier = poseSupplier;
    this.visionConsumer = visionConsumer;
  }

  @Override
  public void periodic() {
    boolean useFrontLeftCam = RobotContainer.operatorBoard.getUseLeftCamera();
    targetTagID = RobotContainer.operatorBoard.getAprilTag();

    Pose3d cameraPose;
    Pose2d robotPose;
    Pose2d currentPose = poseSupplier.get();
    visionUpdates = new ArrayList<>();

    // Need to continuously call getUnprocessedResults for all cameras
    List<PhotonPipelineResult> frontLeftCamUnprocessedResults =
        frontLeftCamera.getAllUnreadResults();
    List<PhotonPipelineResult> frontRightCamUnprocessedResults =
        frontRightCamera.getAllUnreadResults();

    // Values of camera being used
    List<PhotonPipelineResult> unprocessedResults =
        useFrontLeftCam ? frontLeftCamUnprocessedResults : frontRightCamUnprocessedResults;
    Pose3d robotToCameraPose =
        useFrontLeftCam
            ? Constants.Vision.frontLeftCamera3dPos
            : Constants.Vision.frontRightCamera3dPos;

    // skip loop if camera hasn't processed a new frame since last check
    if (unprocessedResults.isEmpty()) {
      return;
    }

    // Cache latest result for use in helper methods
    latestResult = unprocessedResults.get(unprocessedResults.size() - 1);

    // Filter through each unread pipeline result and add to pose estimator with corresponding
    // timestamp
    for (PhotonPipelineResult unprocessedResult : unprocessedResults) {
      // continue if there's no targets
      if (!unprocessedResult.hasTargets()) {
        continue;
      }

      PhotonTrackedTarget target = unprocessedResult.getBestTarget();
      // Continue if the camera doesn't have the right target we're looking for
      if (target.fiducialId != targetTagID) {
        continue;
      }

      Pose3d tagPos = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get();
      double timestamp = unprocessedResult.getTimestampSeconds();

      // Disambiguate and use best pose estimate
      Pose3d cameraPose0 = tagPos.transformBy(target.getBestCameraToTarget().inverse());
      Pose3d cameraPose1 = tagPos.transformBy(target.getAlternateCameraToTarget().inverse());
      Pose2d robotPose0 =
          cameraPose0
              .transformBy(GeomUtil.pose3dToTransform3d(robotToCameraPose).inverse())
              .toPose2d();
      Pose2d robotPose1 =
          cameraPose1
              .transformBy(GeomUtil.pose3dToTransform3d(robotToCameraPose).inverse())
              .toPose2d();

      double projectionError = target.getPoseAmbiguity();

      // Select a pose using projection error and current rotation
      if (projectionError < 0.15) {
        cameraPose = cameraPose0;
        robotPose = robotPose0;
      } else if (Math.abs(robotPose0.getRotation().minus(currentPose.getRotation()).getRadians())
          < Math.abs(robotPose1.getRotation().minus(currentPose.getRotation()).getRadians())) {
        cameraPose = cameraPose0;
        robotPose = robotPose0;
      } else {
        cameraPose = cameraPose1;
        robotPose = robotPose1;
      }

      // Continue with filtering after disambiguation
      if (cameraPose == null || robotPose == null) {
        continue;
      }

      // Use gyro to rotate vision translation vector to be more accurate
      Rotation2d robotThetaError =
          robotPose.getRotation().minus(RobotContainer.swerve.getPose().getRotation());
      Pose2d tagToRobotPose = robotPose.relativeTo(tagPos.toPose2d());
      robotPose =
          tagPos
              .toPose2d()
              .transformBy(GeomUtil.poseToTransform(tagToRobotPose.rotateBy(robotThetaError)));

      // Move on to next camera if robot pose is off the field
      if (robotPose.getX() < -fieldBorderMargin
          || robotPose.getX() > FieldConstants.fieldLength + fieldBorderMargin
          || robotPose.getY() < -fieldBorderMargin
          || robotPose.getY() > FieldConstants.fieldWidth + fieldBorderMargin) {
        continue;
      }

      // Scale xy standard deviation with distance
      double xyStdDev =
          xyStdDevModel.predict(tagPos.getTranslation().getDistance(cameraPose.getTranslation()));

      visionUpdates.add(
          new TimestampedVisionUpdate(
              robotPose,
              timestamp,
              VecBuilder.fill(
                  Constants.Vision.xPosVisionStandardDev * xyStdDev,
                  Constants.Vision.yPosVisionStandardDev * xyStdDev,
                  Constants.Vision.thetaVisionStandardDev)));

      Logger.recordOutput("Vision/Pose Estimate 1", robotPose0);
      Logger.recordOutput("Vision/Pose Estimate 2", robotPose1);
      Logger.recordOutput("Vision/Pose Estimate", robotPose);
    }

    // Apply all vision updates to pose estimator
    visionConsumer.accept(visionUpdates);
  }

  public boolean hasTargetTag() {
    if (latestResult.hasTargets()) {
      for (PhotonTrackedTarget target : latestResult.getTargets()) {
        if (target.fiducialId == targetTagID) {
          return true;
        }
      }
    }
    return false;
  }
}
