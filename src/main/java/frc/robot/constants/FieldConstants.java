package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.*;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
  public static final double fieldLength = 17.548;
  public static final double fieldWidth = 8.052;
  public static final double startingLineX =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line
  public static final double algaeDiameter = Units.inchesToMeters(16);

  public static class Processor {
    public static final Pose2d centerFace =
        new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
  }

  public static class Barge {
    public static final Translation2d farCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
    public static final Translation2d middleCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
    public static final Translation2d closeCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

    // Measured from floor to bottom of cage
    public static final double deepHeight = Units.inchesToMeters(3.125);
    public static final double shallowHeight = Units.inchesToMeters(30.125);
  }

  public static class CoralStation {
    public static final double stationLength = Units.inchesToMeters(79.750);
    public static final Pose2d leftCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(291.176),
            Rotation2d.fromDegrees(90 - 144.011)); // -54
    public static final Pose2d rightCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90)); // 54
  }

  public static class Reef {
    public static final double faceLength = Units.inchesToMeters(36.792600);
    public static final Translation2d center =
        new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
    public static final double faceToZoneLine =
        Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

    public static final Pose2d[] centerFaces =
        new Pose2d[6]; // Starting facing the driver station in clockwise order
    public static final List<Map<ReefLevel, Pose3d>> branchPositions =
        new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise

    static {
      // Initialize faces
      centerFaces[0] =
          new Pose2d(
              Units.inchesToMeters(144.003),
              Units.inchesToMeters(158.500),
              Rotation2d.fromDegrees(180));
      centerFaces[1] =
          new Pose2d(
              Units.inchesToMeters(160.373),
              Units.inchesToMeters(186.857),
              Rotation2d.fromDegrees(120));
      centerFaces[2] =
          new Pose2d(
              Units.inchesToMeters(193.116),
              Units.inchesToMeters(186.858),
              Rotation2d.fromDegrees(60));
      centerFaces[3] =
          new Pose2d(
              Units.inchesToMeters(209.489),
              Units.inchesToMeters(158.502),
              Rotation2d.fromDegrees(0));
      centerFaces[4] =
          new Pose2d(
              Units.inchesToMeters(193.118),
              Units.inchesToMeters(130.145),
              Rotation2d.fromDegrees(-60));
      centerFaces[5] =
          new Pose2d(
              Units.inchesToMeters(160.375),
              Units.inchesToMeters(130.144),
              Rotation2d.fromDegrees(-120));

      // Initialize branch positions
      for (int face = 0; face < 6; face++) {
        Map<ReefLevel, Pose3d> fillRight = new HashMap<>();
        Map<ReefLevel, Pose3d> fillLeft = new HashMap<>();
        for (var level : ReefLevel.values()) {
          Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
          double adjustX = Units.inchesToMeters(30.738);
          double adjustY = Units.inchesToMeters(6.469);

          fillRight.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians())));
          fillLeft.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians())));
        }
        branchPositions.add(fillRight);
        branchPositions.add(fillLeft);
      }
    }
  }

  public static class StagingPositions {
    // Measured from the center of the ice cream
    public static final Pose2d leftIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
    public static final Pose2d middleIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
    public static final Pose2d rightIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
  }

  public enum ReefLevel {
    L1(Units.inchesToMeters(25.0), 0),
    L2(Units.inchesToMeters(31.875), -35),
    L3(Units.inchesToMeters(47.625), -35),
    L4(Units.inchesToMeters(72), -90);

    ReefLevel(double height, double pitch) {
      this.height = height;
      this.pitch = pitch; // in degrees
    }

    public static ReefLevel fromLevel(int level) {
      return Arrays.stream(values())
          .filter(height -> height.ordinal() == level)
          .findFirst()
          .orElse(L4);
    }

    public final double height;
    public final double pitch;
  }

  public static final double aprilTagWidth = Units.inchesToMeters(6.50);
  public static final int aprilTagCount = 22;
  public static final AprilTagFieldLayout aprilTagFieldLayout =
      new AprilTagFieldLayout(
          List.of(
              new AprilTag(
                  1,
                  new Pose3d(
                      16.697198,
                      0.65532,
                      1.4859,
                      new Rotation3d(
                          new Quaternion(0.4539904997395468, 0.0, 0.0, 0.8910065241883678)))),
              new AprilTag(
                  2,
                  new Pose3d(
                      16.697198,
                      7.3964799999999995,
                      1.4859,
                      new Rotation3d(
                          new Quaternion(-0.45399049973954675, -0.0, 0.0, 0.8910065241883679)))),
              new AprilTag(
                  3,
                  new Pose3d(
                      11.560809999999998,
                      8.05561,
                      1.30175,
                      new Rotation3d(
                          new Quaternion(-0.7071067811865475, -0.0, 0.0, 0.7071067811865476)))),
              new AprilTag(
                  4,
                  new Pose3d(
                      9.276079999999999,
                      6.137656,
                      1.8679160000000001,
                      new Rotation3d(
                          new Quaternion(0.9659258262890683, 0.0, 0.25881904510252074, 0.0)))),
              new AprilTag(
                  5,
                  new Pose3d(
                      9.276079999999999,
                      1.914906,
                      1.8679160000000001,
                      new Rotation3d(
                          new Quaternion(0.9659258262890683, 0.0, 0.25881904510252074, 0.0)))),
              new AprilTag(
                  6,
                  new Pose3d(
                      13.474446,
                      3.3063179999999996,
                      0.308102,
                      new Rotation3d(
                          new Quaternion(-0.8660254037844387, -0.0, 0.0, 0.49999999999999994)))),
              new AprilTag(
                  7,
                  new Pose3d(
                      13.890498,
                      4.0259,
                      0.308102,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new AprilTag(
                  8,
                  new Pose3d(
                      13.474446,
                      4.745482,
                      0.308102,
                      new Rotation3d(
                          new Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))),
              new AprilTag(
                  9,
                  new Pose3d(
                      12.643358,
                      4.745482,
                      0.308102,
                      new Rotation3d(
                          new Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386)))),
              new AprilTag(
                  10,
                  new Pose3d(
                      12.227305999999999,
                      4.0259,
                      0.308102,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new AprilTag(
                  11,
                  new Pose3d(
                      12.643358,
                      3.3063179999999996,
                      0.308102,
                      new Rotation3d(
                          new Quaternion(-0.4999999999999998, -0.0, 0.0, 0.8660254037844387)))),
              new AprilTag(
                  12,
                  new Pose3d(
                      0.851154,
                      0.65532,
                      1.4859,
                      new Rotation3d(
                          new Quaternion(0.8910065241883679, 0.0, 0.0, 0.45399049973954675)))),
              new AprilTag(
                  13,
                  new Pose3d(
                      0.851154,
                      7.3964799999999995,
                      1.4859,
                      new Rotation3d(
                          new Quaternion(-0.8910065241883678, -0.0, 0.0, 0.45399049973954686)))),
              new AprilTag(
                  14,
                  new Pose3d(
                      8.272272,
                      6.137656,
                      1.8679160000000001,
                      new Rotation3d(
                          new Quaternion(
                              5.914589856893349e-17,
                              -0.25881904510252074,
                              1.5848095757158825e-17,
                              0.9659258262890683)))),
              new AprilTag(
                  15,
                  new Pose3d(
                      8.272272,
                      1.914906,
                      1.8679160000000001,
                      new Rotation3d(
                          new Quaternion(
                              5.914589856893349e-17,
                              -0.25881904510252074,
                              1.5848095757158825e-17,
                              0.9659258262890683)))),
              new AprilTag(
                  16,
                  new Pose3d(
                      5.9875419999999995,
                      -0.0038099999999999996,
                      1.30175,
                      new Rotation3d(
                          new Quaternion(0.7071067811865476, 0.0, 0.0, 0.7071067811865476)))),
              new AprilTag(
                  17,
                  new Pose3d(
                      4.073905999999999,
                      3.3063179999999996,
                      0.308102,
                      new Rotation3d(
                          new Quaternion(-0.4999999999999998, -0.0, 0.0, 0.8660254037844387)))),
              new AprilTag(
                  18,
                  new Pose3d(
                      3.6576,
                      4.0259,
                      0.308102,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new AprilTag(
                  19,
                  new Pose3d(
                      4.073905999999999,
                      4.745482,
                      0.308102,
                      new Rotation3d(
                          new Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386)))),
              new AprilTag(
                  20,
                  new Pose3d(
                      4.904739999999999,
                      4.745482,
                      0.308102,
                      new Rotation3d(
                          new Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))),
              new AprilTag(
                  21,
                  new Pose3d(
                      5.321046,
                      4.0259,
                      0.308102,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new AprilTag(
                  22,
                  new Pose3d(
                      4.904739999999999,
                      3.3063179999999996,
                      0.308102,
                      new Rotation3d(
                          new Quaternion(-0.8660254037844387, -0.0, 0.0, 0.49999999999999994))))),
          fieldLength,
          fieldWidth);
}
