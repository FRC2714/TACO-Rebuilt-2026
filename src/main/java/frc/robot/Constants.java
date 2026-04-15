// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 15;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 11;
    public static final int kRearRightDrivingCanId = 9;

    public static final int kFrontLeftTurningCanId = 14;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 10;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = true;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 6780;
    public static final double kVortexKv = 565; // rpm/V
  }

  public static final class Intake {
    public static final int kRollerCanId = 16; // change later
    public static final int kConveyorCanId = 4; // change later
    public static final int kPivotCanId = 17;

    public static final double kPivotReduction = 25; // TODO: set to actual gear reduction
    public static final double kPivotThreshold = 1.0; // degrees tolerance for at-setpoint

    public static final double kP = 0.0022; // TODO: tune
    public static final double kD = 0.0;
    public static final double kRetractFeedforward = -0.65; // extra power when going up (tune)
    public static final double kZeroSpeed = -0.35; // slow speed for zeroing (going up)

    public static final class RollerSetpoints {
      public static final double kIntake = -1;
      public static final double kExtake = 1;
      public static final double kStop = 0;
    }

    public static final class ConveyorSetpoints {
      public static final double kIntake = -1;
      public static final double kExtake = 1;
      public static final double kStop = 0;
    }

    // Setpoints in degrees of output shaft rotation. 0 = stowed (all the way up at boot).
    // Positive = deploy down. TODO: tune all values on the robot.
    public static final class PivotSetpoints {
      public static final double kStow = 0.0;

      public static final double kIntake = 271.14;
      public static final double kExtake = 271.14;
      public static final double kAgitate = 200.0;
    }

    public static final class AgitatorConstants {
      public static final double kDelayBeforeAgitating = 1.25; // seconds after flywheel spins up
      public static final int kAgitationCount = 1; // number of stow/deploy cycles
      public static final double kStowDuration = 1.2; // seconds to hold stow
      public static final double kDeployDuration = 0.75; // seconds to hold deploy
    }
  }

  public static final class LimelightConstants {
    public static final String kFrontName = "limelight-front";
    public static final String kBackName = "limelight-back";
    public static final Matrix<N3, N1> m_stateStdDevs = VecBuilder.fill(0.15, 0.15, 0.00001);
    public static final Matrix<N3, N1> m_visionStdDevs = VecBuilder.fill(0.7, 0.7, 999999);
  }

  public static final class AutoAlignConstants {
    public static final double kP = 0.08;
    public static final double kI = 0;
    public static final double kD = 0.005;
    public static final double kAlignTolerance = 0.3; // degrees of tx error
    public static final double kRotationOffsetDeg =
        0.0; // degrees to nudge aim (+ = left, - = right)
  }

  public static final class AutoAimConstants {
    // Passing targets: aimed at the starting line, offset into the trench openings
    public static final Translation2d kBlueLeftTarget =
        new Translation2d(
            frc.robot.FieldConstants.LinesVertical.starting,
            frc.robot.FieldConstants.LinesHorizontal.leftTrenchOpenEnd - 0.6);

    public static final Translation2d kBlueRightTarget =
        new Translation2d(
            frc.robot.FieldConstants.LinesVertical.starting,
            frc.robot.FieldConstants.LinesHorizontal.rightTrenchOpenStart + 0.6);

    public static final Translation2d kRedLeftTarget =
        new Translation2d(
            frc.robot.FieldConstants.fieldLength - frc.robot.FieldConstants.LinesVertical.starting,
            frc.robot.FieldConstants.fieldWidth
                - (frc.robot.FieldConstants.LinesHorizontal.leftTrenchOpenEnd - 0.6));

    public static final Translation2d kRedRightTarget =
        new Translation2d(
            frc.robot.FieldConstants.fieldLength - frc.robot.FieldConstants.LinesVertical.starting,
            frc.robot.FieldConstants.fieldWidth
                - (frc.robot.FieldConstants.LinesHorizontal.rightTrenchOpenStart + 0.6));
  }

  public static final class ShooterConstants {
    public static final int kFeederMotorCanId = 5;
    public static final int kFlywheelMotorCanId = 6;
    public static final int kFlywheelFollowerMotorCanId = 7;
    public static final double kFlywheelDebounceTimeSeconds = 0.1;
    public static final double kLatencyCompensation = 0.1; // seconds
    public static final double kRpmScaleFactor = 0.9; // Scale all lookup RPMs down (tune this)
    public static final double kRpmPerMpsRadial = 45.0; // RPM per m/s moving away from hub (tune)
    public static final double kRpmPerMpsLateral = 80.0; // RPM per m/s moving sideways (tune)
    public static final double kUnjamDuration = 0.5; // seconds to reverse feeder before shooting
    public static final double kHeadingPerMpsLateral =
        5.0; // degrees of extra heading lead per m/s lateral speed (tune)

    public static final class FeederSetpoints {
      public static final double kFeed = 1;
    }

    public static final class FlywheelSetpoints {
      public static final double kShootRpm = 2200; // fallback default
      public static final double kPassRpm = 3750; // fallback default
      public static final double kVelocityTolerance = 300;
    }
  }
}
