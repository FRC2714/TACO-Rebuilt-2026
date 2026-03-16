// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Field;

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
    return new ParallelCommandGroup(
            m_shooter.runShooterCommand(),
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> m_shooter.isFlywheelSpinning.getAsBoolean()),
                new ParallelCommandGroup(
                    m_intake.conveyorCommand(), m_intake.shootingAgitateCommand())))
        .beforeStarting(() -> m_driveSubsystem.setAligning(true))
        .finallyDo(interrupted -> stowAll());
  }

  public Command passingSequence() {
    return new ParallelCommandGroup(
            m_shooter.runPassCommand(),
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> m_shooter.isFlywheelSpinning.getAsBoolean()),
                new ParallelCommandGroup(
                    m_intake.conveyorCommand(), m_intake.shootingAgitateCommand())))
        .finallyDo(interrupted -> stowAll());
  }

  @Override
  public void periodic() {
    Pose2d pose = m_driveSubsystem.getPose();
    Translation2d robotPosition = pose.getTranslation();
    Translation2d goalPosition = Field.getAllianceHub().toTranslation2d();

    double distanceToHub = robotPosition.getDistance(goalPosition);
    SmartDashboard.putNumber("Distance to Hub", distanceToHub);

    m_shooter.calculate(
        robotPosition,
        pose.getRotation(),
        m_driveSubsystem.getFieldRelativeVelocity(),
        goalPosition,
        ShooterConstants.kLatencyCompensation);

    m_driveSubsystem.setTargetHeading(
        m_shooter.getCalculatedHeadingDeg()
            + frc.robot.Constants.AutoAlignConstants.kRotationOffsetDeg);
  }
}
