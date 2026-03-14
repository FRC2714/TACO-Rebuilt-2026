package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class FieldConstants {
  private static final AprilTagFieldLayout layout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  public static final double fieldLength = layout.getFieldLength();
  public static final double fieldWidth = layout.getFieldWidth();

  private static Pose3d tagPose(int id) {
    return layout.getTagPose(id).get();
  }

  public static final class Hub {
    public static final double width = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(72.0);
    public static final double innerWidth = Units.inchesToMeters(41.7);
    public static final double innerHeight = Units.inchesToMeters(56.5);

    // Blue alliance hub (anchored to AprilTag 26)
    public static final Translation3d topCenterPoint =
        new Translation3d(
            tagPose(26).getX() + width / 2.0, fieldWidth / 2.0, height);
    public static final Translation3d innerCenterPoint =
        new Translation3d(
            tagPose(26).getX() + width / 2.0, fieldWidth / 2.0, innerHeight);

    public static final Translation2d nearLeftCorner =
        new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d nearRightCorner =
        new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
    public static final Translation2d farLeftCorner =
        new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d farRightCorner =
        new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

    // Red (opponent) hub (anchored to AprilTag 4)
    public static final Translation3d oppTopCenterPoint =
        new Translation3d(
            tagPose(4).getX() + width / 2.0, fieldWidth / 2.0, height);

    public static final Translation2d oppNearLeftCorner =
        new Translation2d(
            oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d oppNearRightCorner =
        new Translation2d(
            oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
    public static final Translation2d oppFarLeftCorner =
        new Translation2d(
            oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d oppFarRightCorner =
        new Translation2d(
            oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);
  }

  public static final class LinesVertical {
    public static final double center = fieldLength / 2.0;
    public static final double starting = tagPose(26).getX();
    public static final double allianceZone = starting;
    public static final double hubCenter = tagPose(26).getX() + Hub.width / 2.0;
    public static final double neutralZoneNear = center - Units.inchesToMeters(120);
    public static final double neutralZoneFar = center + Units.inchesToMeters(120);
    public static final double oppHubCenter = tagPose(4).getX() + Hub.width / 2.0;
    public static final double oppAllianceZone = tagPose(10).getX();
  }

  public static final class LinesHorizontal {
    public static final double center = fieldWidth / 2.0;
    public static final double rightBumpStart = Hub.nearRightCorner.getY();
    public static final double rightBumpEnd = rightBumpStart - RightBump.width;
    public static final double rightTrenchOpenStart = rightBumpEnd - Units.inchesToMeters(12.0);
    public static final double rightTrenchOpenEnd = 0;
    public static final double leftBumpEnd = Hub.nearLeftCorner.getY();
    public static final double leftBumpStart = leftBumpEnd + LeftBump.width;
    public static final double leftTrenchOpenEnd = leftBumpStart + Units.inchesToMeters(12.0);
    public static final double leftTrenchOpenStart = fieldWidth;
  }

  public static final class LeftBump {
    public static final double width = Units.inchesToMeters(73.0);
    public static final double height = Units.inchesToMeters(6.513);
    public static final double depth = Units.inchesToMeters(44.4);
  }

  public static final class RightBump {
    public static final double width = Units.inchesToMeters(73.0);
    public static final double height = Units.inchesToMeters(6.513);
    public static final double depth = Units.inchesToMeters(44.4);
  }

  public static final class Tower {
    public static final double width = Units.inchesToMeters(49.25);
    public static final double depth = Units.inchesToMeters(45.0);
    public static final double height = Units.inchesToMeters(78.25);
    public static final double innerOpeningWidth = Units.inchesToMeters(32.25);
    public static final double frontFaceX = Units.inchesToMeters(43.51);
    public static final double lowRungHeight = Units.inchesToMeters(27.0);
    public static final double midRungHeight = Units.inchesToMeters(45.0);
    public static final double highRungHeight = Units.inchesToMeters(63.0);

    // Blue tower (anchored to AprilTag 31)
    public static final Translation2d centerPoint =
        new Translation2d(frontFaceX, tagPose(31).getY());

    // Red tower (anchored to AprilTag 15)
    public static final Translation2d oppCenterPoint =
        new Translation2d(fieldLength - frontFaceX, tagPose(15).getY());
  }

  public static final class Depot {
    public static final double width = Units.inchesToMeters(42.0);
    public static final double depth = Units.inchesToMeters(27.0);
    public static final double height = Units.inchesToMeters(1.125);
    public static final double distanceFromCenterY = Units.inchesToMeters(75.93);
    public static final Translation3d depotCenter =
        new Translation3d(depth, fieldWidth / 2.0 + distanceFromCenterY, height);
  }

  public static final class Outpost {
    public static final double width = Units.inchesToMeters(31.8);
    public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
    public static final double height = Units.inchesToMeters(7.0);
    public static final Translation2d centerPoint = new Translation2d(0, tagPose(29).getY());
  }
}
