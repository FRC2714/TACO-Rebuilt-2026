// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Field;
import frc.robot.FieldConstants;

public class Superstructure extends SubsystemBase {
  private DriveSubsystem m_driveSubsystem;
  private Intake m_intake;
  private Shooter m_shooter;

  public Superstructure(DriveSubsystem driveSubsystem, Intake intake, Shooter shooter) {
    m_driveSubsystem = driveSubsystem;
    m_intake = intake;
    m_shooter = shooter;
  }

  /** Stow everything: stop shooter, stop intake, stop aligning. */
  private void stowAll() {
    m_shooter.stopAll();
    m_intake.stopAll();
    m_driveSubsystem.setAligning(false);
    m_driveSubsystem.setShooting(false);
  }

  public Command stowCommand() {
    return Commands.runOnce(this::stowAll).withName("StowAll");
  }

  public Command shooterSequence() {
    return shooterSequence(true, true);
  }

  public Command shooterSequence(boolean agitate) {
    return shooterSequence(agitate, true);
  }

  public Command shooterSequence(boolean agitate, boolean align) {
    return shooterSequence(agitate, align, -1, -1, -1, -1);
  }

  public Command shooterSequence(
      boolean agitate, double delay, double stowTime, double deployTime, int count) {
    return shooterSequence(agitate, true, delay, stowTime, deployTime, count);
  }

  public Command shooterSequence(
      boolean agitate, boolean align, double delay, double stowTime, double deployTime, int count) {
    Command feedGroup =
        new SequentialCommandGroup(
            new WaitUntilCommand(() -> m_shooter.isFlywheelSpinning.getAsBoolean()),
            agitate
                ? new ParallelCommandGroup(
                    m_intake.conveyorCommand(),
                    m_intake.shootingRollerCommand(),
                    m_intake.shootingAgitateCommand(
                        delay >= 0 ? delay : AgitatorDefaults.kDelay,
                        stowTime >= 0 ? stowTime : AgitatorDefaults.kStow,
                        deployTime >= 0 ? deployTime : AgitatorDefaults.kDeploy,
                        count >= 0 ? count : AgitatorDefaults.kCount))
                : new ParallelCommandGroup(
                    m_intake.conveyorCommand(), m_intake.shootingRollerCommand()));

    ParallelCommandGroup group;
    if (align) {
      group =
          new ParallelCommandGroup(
              m_shooter.runShooterCommand(),
              m_driveSubsystem.run(() -> m_driveSubsystem.drive(0, 0, 0, true)),
              feedGroup);
    } else {
      group = new ParallelCommandGroup(m_shooter.runShooterCommand(), feedGroup);
    }

    return group
        .beforeStarting(
            () -> {
              if (align) m_driveSubsystem.setAligning(true);
            })
        .finallyDo(interrupted -> stowAll());
  }

  public Command passingSequence() {
    return passingSequence(true);
  }

  public Command passingSequence(boolean agitate) {
    return passingSequence(agitate, -1, -1, -1, -1);
  }

  public Command passingSequence(
      boolean agitate, double delay, double stowTime, double deployTime, int count) {
    return new ParallelCommandGroup(
            m_shooter.runPassCommand(),
            m_driveSubsystem.run(() -> m_driveSubsystem.drive(0, 0, 0, true)),
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> m_shooter.isFlywheelSpinning.getAsBoolean()),
                agitate
                    ? new ParallelCommandGroup(
                        m_intake.conveyorCommand(),
                        m_intake.shootingRollerCommand(),
                        m_intake.shootingAgitateCommand(
                            delay >= 0 ? delay : AgitatorDefaults.kDelay,
                            stowTime >= 0 ? stowTime : AgitatorDefaults.kStow,
                            deployTime >= 0 ? deployTime : AgitatorDefaults.kDeploy,
                            count >= 0 ? count : AgitatorDefaults.kCount))
                    : new ParallelCommandGroup(
                        m_intake.conveyorCommand(), m_intake.shootingRollerCommand())))
        .finallyDo(interrupted -> stowAll());
  }

  private static final class AgitatorDefaults {
    static final double kDelay = frc.robot.Constants.Intake.AgitatorConstants.kDelayBeforeAgitating;
    static final double kStow = frc.robot.Constants.Intake.AgitatorConstants.kStowDuration;
    static final double kDeploy = frc.robot.Constants.Intake.AgitatorConstants.kDeployDuration;
    static final int kCount = frc.robot.Constants.Intake.AgitatorConstants.kAgitationCount;
  }

  /** Select the appropriate passing target based on which side of the hub the robot is on. */
  private Translation2d getPassingTarget() {
    double robotY = m_driveSubsystem.getPose().getY();
    double centerY = FieldConstants.LinesHorizontal.center;

    if (Field.isRed()) {
      return robotY < centerY ? AutoAimConstants.kRedLeftTarget : AutoAimConstants.kRedRightTarget;
    } else {
      return robotY < centerY
          ? AutoAimConstants.kBlueRightTarget
          : AutoAimConstants.kBlueLeftTarget;
    }
  }

  // Phase shift timing
  private static final int[] PHASE_SHIFT_TIMES = {130, 105, 80, 55, 30};
  private static final double PHASE_SHIFT_WARNING_SECONDS = 5.0;

  // Randomized for practice without FMS — re-randomized each boot
  private final boolean practiceRedInactiveFirst = Math.random() < 0.5;

  private double getTimeUntilNextPhaseShift(double matchTime) {
    if (!DriverStation.isTeleopEnabled()) return -1;
    for (int boundary : PHASE_SHIFT_TIMES) {
      if (matchTime >= boundary) return matchTime - boundary;
    }
    return -1;
  }

  private boolean isPhaseShiftWarning(double matchTime) {
    double timeUntil = getTimeUntilNextPhaseShift(matchTime);
    return timeUntil > 0 && timeUntil <= PHASE_SHIFT_WARNING_SECONDS;
  }

  private boolean getPhaseShiftDashboardValue(double matchTime, boolean hubActive) {
    if (!isPhaseShiftWarning(matchTime)) return hubActive;
    // Flash at 4Hz during warning window
    return ((int) Math.floor(matchTime * 4.0)) % 2 == 0;
  }

  /**
   * Returns true if our alliance's hub is currently active. Game data "R" = Red hub inactive first,
   * "B" = Blue hub inactive first. Shift1Active alternates: active in shifts 1,3; inactive in 2,4.
   */
  private boolean isOurHubActive() {
    if (DriverStation.isAutonomousEnabled()) return true;
    if (!DriverStation.isTeleopEnabled()) return false;

    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();

    boolean redInactiveFirst;
    if (gameData.isEmpty()) {
      // No FMS: use randomized practice result
      redInactiveFirst = practiceRedInactiveFirst;
    } else {
      switch (gameData.charAt(0)) {
        case 'R':
          redInactiveFirst = true;
          break;
        case 'B':
          redInactiveFirst = false;
          break;
        default:
          return true; // corrupt data, safe fallback
      }
    }

    // shift1Active: true if our hub is active during shifts 1 & 3
    boolean shift1Active = Field.isRed() ? !redInactiveFirst : redInactiveFirst;

    if (matchTime > 130) {
      return true; // transition shift
    } else if (matchTime > 105) {
      return shift1Active;
    } else if (matchTime > 80) {
      return !shift1Active;
    } else if (matchTime > 55) {
      return shift1Active;
    } else if (matchTime > 30) {
      return !shift1Active;
    } else {
      return true; // endgame, always active
    }
  }

  @Override
  public void periodic() {
    Pose2d pose = m_driveSubsystem.getPose();
    Translation2d robotPosition = pose.getTranslation();
    boolean inAllianceZone = m_driveSubsystem.isInAllianceZone();
    boolean hubActive = isOurHubActive();

    // In alliance zone: aim at hub. Outside: aim at passing target.
    Translation2d target =
        inAllianceZone ? Field.getAllianceHub().toTranslation2d() : getPassingTarget();

    double matchTime = DriverStation.getMatchTime();
    double timeToNextShift = getTimeUntilNextPhaseShift(matchTime);

    double distanceToTarget = robotPosition.getDistance(target);
    SmartDashboard.putNumber("Distance to Hub", distanceToTarget);
    SmartDashboard.putBoolean("In Alliance Zone", inAllianceZone);
    SmartDashboard.putNumber("Match Time", matchTime);
    SmartDashboard.putBoolean(
        "Match/Hub Active", getPhaseShiftDashboardValue(matchTime, hubActive));
    SmartDashboard.putString(
        "Match/Time Until Next Phase Shift",
        timeToNextShift >= 0 ? String.format("%.2f", timeToNextShift) : "N/A");

    m_shooter.calculate(
        robotPosition,
        pose.getRotation(),
        m_driveSubsystem.getFieldRelativeVelocity(),
        target,
        ShooterConstants.kLatencyCompensation);

    m_driveSubsystem.setTargetHeading(
        m_shooter.getCalculatedHeadingDeg()
            + frc.robot.Constants.AutoAlignConstants.kRotationOffsetDeg);

    // Auto pre-spin: only if hub active, in alliance zone, teleop, and intaked recently
    boolean shouldPreSpin =
        hubActive
            && inAllianceZone
            && DriverStation.isTeleopEnabled()
            && m_intake.hasIntakedRecently(6.0);
    m_shooter.setAutoPreSpin(shouldPreSpin);
    SmartDashboard.putBoolean("Match/Auto PreSpin", shouldPreSpin);
  }
}
