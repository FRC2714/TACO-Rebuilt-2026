// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Superstructure extends SubsystemBase {
  private DriveSubsystem m_driveSubsystem;
  private Intake m_intake;
  private Shooter m_shooter;

  public Superstructure(DriveSubsystem driveSubsystem, Intake intake, Shooter shooter) {
    m_driveSubsystem = driveSubsystem;
    m_intake = intake;
    m_shooter = shooter;
  }

  public Command shooterSequence() {
    return new ParallelCommandGroup(
            m_shooter.runShooterCommand(),
            new SequentialCommandGroup(
                new WaitUntilCommand(
                    () ->
                        m_shooter.isFlywheelSpinning.getAsBoolean()
                            && m_driveSubsystem.isAligned()),
                m_intake.conveyorCommand()))
        .beforeStarting(() -> m_driveSubsystem.setAligning(true))
        .finallyDo(
            () -> {
              m_driveSubsystem.setAligning(false);
              m_shooter.stopAll();
            });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
