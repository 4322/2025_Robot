package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;

public class FieldConstants {

  public static final double fieldLength = 17.55;
  public static final double fieldWidth = 8.05;

  // AprilTag constants
  public static final double aprilTagWidth = Units.inchesToMeters(8.12500);
  public static final AprilTagFieldLayout aprilTags =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
}
