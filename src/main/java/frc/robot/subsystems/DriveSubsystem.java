// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.utils.LimelightHelpers;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft =
      new MAXSwerveModule(
          DriveConstants.kFrontLeftDrivingCanId,
          DriveConstants.kFrontLeftTurningCanId,
          DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight =
      new MAXSwerveModule(
          DriveConstants.kFrontRightDrivingCanId,
          DriveConstants.kFrontRightTurningCanId,
          DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft =
      new MAXSwerveModule(
          DriveConstants.kRearLeftDrivingCanId,
          DriveConstants.kRearLeftTurningCanId,
          DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight =
      new MAXSwerveModule(
          DriveConstants.kRearRightDrivingCanId,
          DriveConstants.kRearRightTurningCanId,
          DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final Canandgyro m_gyro = new Canandgyro(0);

  private final Field2d m_field2d = new Field2d();

  private double m_driverHeadingOffsetDeg = 0.0;

  private boolean aligning = false;
  private boolean shooting = false;
  private double alignRotSpeed = 0.0;
  private double targetHeadingDeg = 0.0;
  private final PIDController alignPID =
      new PIDController(AutoAlignConstants.kP, AutoAlignConstants.kI, AutoAlignConstants.kD);

  // Pose estimator with vision fusion
  SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          getHeading(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          },
          new Pose2d(),
          LimelightConstants.m_stateStdDevs,
          LimelightConstants.m_visionStdDevs);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    SmartDashboard.putData("Field", m_field2d);

    LimelightHelpers.SetIMUMode(LimelightConstants.kFrontName, 4);
    LimelightHelpers.SetIMUMode(LimelightConstants.kBackName, 4);

    RobotConfig config =
        new RobotConfig(
            30,
            6.0,
            new ModuleConfig(
                ModuleConstants.kWheelDiameterMeters / 2,
                DriveConstants.kMaxSpeedMetersPerSecond,
                1.2,
                DCMotor.getNeoVortex(1).withReduction(ModuleConstants.kDrivingMotorReduction),
                60,
                1),
            new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2),
            new Translation2d(DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2),
            new Translation2d(-DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2),
            new Translation2d(-DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2));

    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        () ->
            DriveConstants.kDriveKinematics.toChassisSpeeds(
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState()),
        (speeds, feedforwards) -> {
          var states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
          setModuleStates(states);
        },
        new PPHolonomicDriveController(
            new PIDConstants(AutoConstants.kPXController, 0, 0),
            new PIDConstants(AutoConstants.kPThetaController, 0, 0)),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
  }

  @Override
  public void periodic() {
    m_poseEstimator.update(
        getHeading(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });

    double poseHeadingDeg = getPose().getRotation().getDegrees();
    LimelightHelpers.SetRobotOrientation(
        LimelightConstants.kFrontName, poseHeadingDeg, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(
        LimelightConstants.kBackName, poseHeadingDeg, 0, 0, 0, 0, 0);
    LimelightHelpers.Flush();

    double omegaRps = Units.degreesToRotations(getTurnRate());

    var frontMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.kFrontName);

    if (Math.abs(omegaRps) < 1
        && frontMeasurement != null
        && frontMeasurement.tagCount > 0
        && isValidPose(frontMeasurement.pose)) {
      double xyStdDev = 0.7 * (1 + frontMeasurement.avgTagDist * 0.5);
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, 9999999));
      m_poseEstimator.addVisionMeasurement(
          frontMeasurement.pose, frontMeasurement.timestampSeconds);
    }

    var backMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.kBackName);

    if (Math.abs(omegaRps) < 1
        && backMeasurement != null
        && backMeasurement.tagCount > 0
        && isValidPose(backMeasurement.pose)) {
      double xyStdDev = 0.7 * (1 + backMeasurement.avgTagDist * 0.5);
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, 9999999));
      m_poseEstimator.addVisionMeasurement(backMeasurement.pose, backMeasurement.timestampSeconds);
    }

    m_field2d.setRobotPose(getPose());

    if (aligning || shooting) {
      double currentHeading = getPose().getRotation().getDegrees();
      double headingError = targetHeadingDeg - currentHeading;
      // Normalize to [-180, 180]
      headingError = ((headingError + 180) % 360 + 360) % 360 - 180;
      alignRotSpeed = alignPID.calculate(-headingError, 0);
    } else {
      alignRotSpeed = 0.0;
    }

    SmartDashboard.putBoolean("AutoAlign/Aligning", aligning);
    SmartDashboard.putBoolean("AutoAlign/Aligned", isAligned());
    SmartDashboard.putNumber("AutoAlign/TargetHeading", targetHeadingDeg);
    SmartDashboard.putNumber(
        "AutoAlign/HeadingError", targetHeadingDeg - getPose().getRotation().getDegrees());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        getHeading(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    double driverRelativeHeading = getPose().getRotation().getDegrees() - m_driverHeadingOffsetDeg;

    if (aligning || shooting) {
      rotDelivered = alignRotSpeed;
    }

    // X-lock when shooting and stationary for defense resistance
    boolean driverStationary = Math.abs(xSpeed) < 0.05 && Math.abs(ySpeed) < 0.05;
    if (shooting && driverStationary) {
      setX();
      return;
    }

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    Rotation2d.fromDegrees(driverRelativeHeading))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Returns the robot's field-relative velocity from swerve module states. */
  public Translation2d getFieldRelativeVelocity() {
    ChassisSpeeds robotSpeeds =
        DriveConstants.kDriveKinematics.toChassisSpeeds(
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState());
    return new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
        .rotateBy(getPose().getRotation());
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void setAligning(boolean aligning) {
    this.aligning = aligning;
    if (!aligning) {
      alignPID.reset();
    }
  }

  public void setShooting(boolean shooting) {
    this.shooting = shooting;
    if (!shooting) {
      alignPID.reset();
    }
  }

  public void setTargetHeading(double headingDeg) {
    this.targetHeadingDeg = headingDeg;
  }

  public boolean isAligned() {
    double headingError = targetHeadingDeg - getPose().getRotation().getDegrees();
    headingError = ((headingError + 180) % 360 + 360) % 360 - 180;
    return Math.abs(headingError) <= AutoAlignConstants.kAlignTolerance;
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Sets the current heading as the driver's forward direction. */
  public void zeroDriverHeading() {
    m_driverHeadingOffsetDeg = getPose().getRotation().getDegrees();
  }

  /** Resets pose to origin and re-seeds Limelight IMU. */
  public void zeroPose() {
    Pose2d pose = new Pose2d();
    m_gyro.setYaw(0);
    m_poseEstimator.resetPosition(
        pose.getRotation(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);

    // Seed orientation, then switch to fused IMU mode after a brief delay
    LimelightHelpers.SetRobotOrientation(LimelightConstants.kFrontName, 0, 0, 0, 0, 0, 0);
    LimelightHelpers.SetIMUMode(LimelightConstants.kFrontName, 1);
    LimelightHelpers.SetRobotOrientation(LimelightConstants.kBackName, 0, 0, 0, 0, 0, 0);
    LimelightHelpers.SetIMUMode(LimelightConstants.kBackName, 1);

    edu.wpi.first.wpilibj2.command.Commands.waitSeconds(0.1)
        .andThen(
            edu.wpi.first.wpilibj2.command.Commands.runOnce(
                () -> {
                  LimelightHelpers.SetIMUMode(LimelightConstants.kFrontName, 4);
                  LimelightHelpers.SetIMUMode(LimelightConstants.kBackName, 4);
                }))
        .schedule();
  }

  /** Returns true if the pose is within the FRC field boundaries and not at the origin. */
  private boolean isValidPose(Pose2d pose) {
    double x = pose.getX();
    double y = pose.getY();
    // Reject poses at origin (default/uninitialized) or outside field bounds
    return x > 0.5 && x < 16.0 && y > 0.5 && y < 8.0;
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading as a Rotation2d
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  /**
   * Returns the heading of the robot in degrees.
   *
   * @return the robot's heading in degrees
   */
  public double getHeadingDegrees() {
    return Units.rotationsToDegrees(m_gyro.getYaw());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return (m_gyro.getAngularVelocityYaw() * 360 * (DriveConstants.kGyroReversed ? -1.0 : 1.0));
  }
}
