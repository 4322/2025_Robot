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

  private PolynomialRegression thetaStdDevModel =
      new PolynomialRegression(
          new double[] {
            0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
            3.223358, 4.093358, 4.726358
          },
          new double[] {0.008, 0.027, 0.015, 0.044, 0.04, 0.078, 0.049, 0.027, 0.059, 0.029, 0.068},
          1);

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
