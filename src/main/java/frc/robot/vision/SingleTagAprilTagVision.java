package frc.robot.vision;

import static frc.robot.constants.FieldConstants.aprilTagFieldLayout;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
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
  private PhotonCamera backCamera;

  private static final double fieldBorderMargin = 0.5;
  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = (x) -> {};
  private List<TimestampedVisionUpdate> visionUpdates;
  private Supplier<Pose2d> poseSupplier = () -> new Pose2d();
  private PhotonPipelineResult latestResult = new PhotonPipelineResult();
  private int targetTagID;
  private boolean useFrontLeftCam;
  private Pose3d robotToCameraPose;
  private List<PhotonPipelineResult> unprocessedResults;
  private List<PhotonPipelineResult> backCamUnprocessedResults;
  private List<PhotonPipelineResult> leftCamUnprocessedResults;
  private List<PhotonPipelineResult> rightCamUnprocessedResults;

  private PolynomialRegression xyStdDevModel =
      new PolynomialRegression(
          new double[] {
            0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
            3.223358, 4.093358, 4.726358
          },
          new double[] {0.005, 0.0135, 0.016, 0.038, 0.0515, 0.0925, 0.12, 0.14, 0.17, 0.27, 0.38},
          2);

  private PolynomialRegression thetaStdDevModel =
      new PolynomialRegression(
          new double[] {
            0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
            3.223358, 4.093358, 4.726358
          },
          new double[] {0.008, 0.027, 0.015, 0.044, 0.04, 0.078, 0.049, 0.027, 0.059, 0.029, 0.068},
          1);

  public SingleTagAprilTagVision(
      PhotonCamera frontLeftCamera, PhotonCamera frontRightCamera, PhotonCamera backCamera) {
    this.frontLeftCamera = frontLeftCamera;
    this.frontRightCamera = frontRightCamera;
    this.backCamera = backCamera;
  }

  public void setDataInterfaces(
      Supplier<Pose2d> poseSupplier, Consumer<List<TimestampedVisionUpdate>> visionConsumer) {
    this.poseSupplier = poseSupplier;
    this.visionConsumer = visionConsumer;
  }

  @Override
  public void periodic() {
    long startLoopMs = RobotController.getFPGATime();

    // Always call camera results once per loop to clear fifo queue
    backCamUnprocessedResults = backCamera.getAllUnreadResults();
    leftCamUnprocessedResults = frontLeftCamera.getAllUnreadResults();
    rightCamUnprocessedResults = frontRightCamera.getAllUnreadResults();

    if (RobotContainer.autoFeedRequested) {
      targetTagID = RobotContainer.coralStationTagID;
      unprocessedResults = backCamUnprocessedResults;
      robotToCameraPose = Constants.Vision.backCamera3dPos;
    } else {
      targetTagID = RobotContainer.operatorBoard.getAprilTag();
      useFrontLeftCam = RobotContainer.operatorBoard.getUseLeftCamera();
      unprocessedResults = useFrontLeftCam ? leftCamUnprocessedResults : rightCamUnprocessedResults;
      robotToCameraPose =
          useFrontLeftCam
              ? Constants.Vision.frontLeftCamera3dPos
              : Constants.Vision.frontRightCamera3dPos;
    }

    Pose3d cameraPose;
    Pose2d robotPose;
    Pose2d currentPose = poseSupplier.get();
    visionUpdates = new ArrayList<>();

    Logger.recordOutput("Vision/Frame Count", unprocessedResults.size());

    // skip loop if camera hasn't processed a new frame since last check
    // Assume max frames(20) in FIFO queue to be switching camera so discard initial results
    if (unprocessedResults.isEmpty() || unprocessedResults.size() == 20) {
      return;
    }

    // Filter through each unread pipeline result and add to pose estimator with corresponding
    // timestamp
    for (PhotonPipelineResult unprocessedResult : unprocessedResults) {
      // // Cache latest result for use in helper methods. Make sure to get latest frame
      if (unprocessedResult.getTimestampSeconds() > latestResult.getTimestampSeconds()) {
        latestResult = unprocessedResult;
      }

      // continue if there's no targets
      if (!unprocessedResult.hasTargets()) {
        continue;
      }

      PhotonTrackedTarget target = null;
      for (PhotonTrackedTarget trackedTarget : unprocessedResult.getTargets()) {
        if (trackedTarget.fiducialId == targetTagID) {
          target = trackedTarget;
        }
      }

      // Continue if the camera doesn't have the right target we're looking for
      if (target == null) {
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

      // Use gyro to rotate vision translation vector for greater accuracy
      Rotation2d robotThetaError =
          RobotContainer.swerve.getPose().getRotation().minus(robotPose.getRotation());
      // Account for rotation discontinuity from bound (-179,180]
      if (Math.abs(robotThetaError.getRadians()) > Math.PI) {
        double minThetaError =
            robotThetaError.getDegrees() + (Math.signum(robotThetaError.getDegrees()) * -360);
        robotThetaError = Rotation2d.fromDegrees(minThetaError);
      }
      Pose2d tagToRobotPose = robotPose.relativeTo(tagPos.toPose2d());
      robotPose =
          tagPos
              .toPose2d()
              .transformBy(GeomUtil.poseToTransform(tagToRobotPose.rotateBy(robotThetaError)));

      Logger.recordOutput("Vision/Pose Estimate 1", robotPose0);
      Logger.recordOutput("Vision/Pose Estimate 2", robotPose1);
      Logger.recordOutput("Vision/Pose Estimate", robotPose);

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
      double thetaStdDev =
          thetaStdDevModel.predict(
              tagPos.getTranslation().getDistance(cameraPose.getTranslation()));

      // make sure standard deviations aren't negative
      if (xyStdDev < 0) {
        xyStdDev = 0.00001;
      }

      if (thetaStdDev < 0) {
        thetaStdDev = 0.00001;
      }

      double latencyMs = RobotController.getTime() / 1000.0 - timestamp * 1000;

      // TODO: Fix timesync server so we don't have to do this
      if (latencyMs < 0) {
        timestamp += ((25 - latencyMs) / 1000);
      }

      visionUpdates.add(
          new TimestampedVisionUpdate(
              robotPose,
              timestamp,
              VecBuilder.fill(
                  Constants.Vision.xPosVisionStandardDev * xyStdDev,
                  Constants.Vision.yPosVisionStandardDev * xyStdDev,
                  4322)));
      Logger.recordOutput("Vision/LatencyMs", latencyMs);
      Logger.recordOutput("Vision/XPosStandDev", Constants.Vision.xPosVisionStandardDev * xyStdDev);
      Logger.recordOutput("Vision/YPosStandDev", Constants.Vision.yPosVisionStandardDev * xyStdDev);
      Logger.recordOutput(
          "Vision/ThetaStandDev", Constants.Vision.thetaVisionStandardDev * thetaStdDev);
    }

    Logger.recordOutput("Vision/hasTargetTag", hasTargetTag());

    // Apply all vision updates to pose estimator
    visionConsumer.accept(visionUpdates);

    Logger.recordOutput(
        "Loop/SingleTagAprilTagVisionMs", (RobotController.getFPGATime() - startLoopMs) / 1000.0);
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
