package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private SparkFlex m_pivot = new SparkFlex(Constants.Intake.kPivotCanId, MotorType.kBrushless);
  private RelativeEncoder m_pivotEncoder = m_pivot.getEncoder();
  private SparkClosedLoopController m_pivotController = m_pivot.getClosedLoopController();

  private SparkFlex m_roller = new SparkFlex(Constants.Intake.kRollerCanId, MotorType.kBrushless);
  private SparkFlex m_conveyor =
      new SparkFlex(Constants.Intake.kConveyorCanId, MotorType.kBrushless);

  private SparkSim rollerSim;
  private SparkSim conveyorSim;

  private double pivotSetpoint = Constants.Intake.PivotSetpoints.kStow;

  private enum RollerSetpoints {
    INTAKE,
    EXTAKE,
    STOP
  }

  private enum ConveyorSetpoints {
    INTAKE,
    EXTAKE,
    STOP
  }

  public Intake() {
    m_pivot.configure(
        Configs.Intake.pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_roller.configure(
        Configs.Intake.rollerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_conveyor.configure(
        Configs.Intake.conveyorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Zero the relative encoder — intake always starts stowed
    m_pivotEncoder.setPosition(0);

    DCMotor motorModel = DCMotor.getNEO(1);
    rollerSim = new SparkSim(m_roller, motorModel);
    conveyorSim = new SparkSim(m_conveyor, motorModel);
  }

  private void setRollerSpeed(RollerSetpoints setpoint) {
    double speed;
    switch (setpoint) {
      case INTAKE:
        speed = Constants.Intake.RollerSetpoints.kIntake;
        break;
      case EXTAKE:
        speed = Constants.Intake.RollerSetpoints.kExtake;
        break;
      case STOP:
        speed = Constants.Intake.RollerSetpoints.kStop;
        break;
      default:
        return;
    }
    m_roller.set(speed);
  }

  private void setConveyorSpeed(ConveyorSetpoints setpoint) {
    double speed;
    switch (setpoint) {
      case INTAKE:
        speed = Constants.Intake.ConveyorSetpoints.kIntake;
        break;
      case EXTAKE:
        speed = Constants.Intake.ConveyorSetpoints.kExtake;
        break;
      case STOP:
        speed = Constants.Intake.ConveyorSetpoints.kStop;
        break;
      default:
        return;
    }
    m_conveyor.set(speed);
  }

  /** Set the pivot to a position setpoint in degrees using closed-loop control. */
  private void setPivotPosition(double positionDeg) {
    pivotSetpoint = positionDeg;
    // Add feedforward boost when retracting (going up / toward 0)
    double arbFf =
        positionDeg < m_pivotEncoder.getPosition() ? Constants.Intake.kRetractFeedforward : 0.0;
    m_pivotController.setSetpoint(
        positionDeg, ControlType.kPosition, com.revrobotics.spark.ClosedLoopSlot.kSlot0, arbFf);
  }

  /** Returns true if the pivot is within tolerance of the current setpoint. */
  public boolean pivotAtSetpoint() {
    return Math.abs(m_pivotEncoder.getPosition() - pivotSetpoint)
        < Constants.Intake.kPivotThreshold;
  }

  /** Stow pivot and stop all motors. */
  public void stopAll() {
    setPivotPosition(Constants.Intake.PivotSetpoints.kStow);
    setRollerSpeed(RollerSetpoints.STOP);
    setConveyorSpeed(ConveyorSetpoints.STOP);
  }

  public Command intakeCommand() {
    double rollerThreshold =
        Constants.Intake.PivotSetpoints.kIntake * 0.3; // 30% of travel before rollers start
    return this.run(
            () -> {
              setPivotPosition(Constants.Intake.PivotSetpoints.kIntake);
              setConveyorSpeed(ConveyorSetpoints.INTAKE);
              if (m_pivotEncoder.getPosition() >= rollerThreshold) {
                setRollerSpeed(RollerSetpoints.INTAKE);
              } else {
                setRollerSpeed(RollerSetpoints.STOP);
              }
            })
        .finallyDo(
            () -> {
              setRollerSpeed(RollerSetpoints.STOP);
              setConveyorSpeed(ConveyorSetpoints.STOP);
            })
        .withName("Intaking");
  }

  public Command deployIntake() {
    return this.runOnce(() -> setPivotPosition(Constants.Intake.PivotSetpoints.kIntake));
  }

  public Command conveyorCommand() {
    return this.startEnd(
            () -> setConveyorSpeed(ConveyorSetpoints.INTAKE),
            () -> setConveyorSpeed(ConveyorSetpoints.STOP))
        .withName("Conveyoring");
  }

  /** Runs intake rollers without claiming Intake subsystem. For use in parallel with conveyor. */
  public Command shootingRollerCommand() {
    return Commands.startEnd(
            () -> setRollerSpeed(RollerSetpoints.INTAKE),
            () -> setRollerSpeed(RollerSetpoints.STOP))
        .withName("ShootingRollers");
  }

  public Command extakeCommand() {
    return this.startEnd(
        () -> {
          setPivotPosition(Constants.Intake.PivotSetpoints.kExtake);
          setRollerSpeed(RollerSetpoints.EXTAKE);
          setConveyorSpeed(ConveyorSetpoints.EXTAKE);
        },
        () -> {
          setRollerSpeed(RollerSetpoints.STOP);
          setConveyorSpeed(ConveyorSetpoints.STOP);
        });
  }

  /** Single agitation cycle: stow with rollers intaking. No deploy — just one push. */
  public Command agitateCommand() {
    double stowTime = Constants.Intake.AgitatorConstants.kStowDuration;

    return this.runOnce(
            () -> {
              setPivotPosition(Constants.Intake.PivotSetpoints.kStow);
              setRollerSpeed(RollerSetpoints.INTAKE);
            })
        .andThen(Commands.waitSeconds(stowTime))
        .finallyDo(() -> setRollerSpeed(RollerSetpoints.STOP))
        .withName("Agitating");
  }

  /** Pivot-only agitation for use during shooting. Does not claim Intake subsystem. */
  public Command shootingAgitateCommand() {
    return shootingAgitateCommand(
        Constants.Intake.AgitatorConstants.kDelayBeforeAgitating,
        Constants.Intake.AgitatorConstants.kStowDuration,
        Constants.Intake.AgitatorConstants.kDeployDuration,
        Constants.Intake.AgitatorConstants.kAgitationCount);
  }

  public Command shootingAgitateCommand(
      double delay, double stowTime, double deployTime, int count) {
    Command agitation = Commands.none();
    for (int i = 0; i < count; i++) {
      agitation =
          agitation
              .andThen(
                  Commands.runOnce(() -> setPivotPosition(Constants.Intake.PivotSetpoints.kStow)))
              .andThen(Commands.waitSeconds(stowTime))
              .andThen(
                  Commands.runOnce(
                      () -> setPivotPosition(Constants.Intake.PivotSetpoints.kAgitate)))
              .andThen(Commands.waitSeconds(deployTime));
    }

    return Commands.waitSeconds(delay).andThen(agitation).withName("ShootingAgitate");
  }

  public Command stowCommand() {
    return this.run(this::stopAll);
  }

  /** Slowly drives pivot up until it stalls, then zeros the encoder. */
  public Command zeroCommand() {
    return this.run(
            () -> {
              setRollerSpeed(RollerSetpoints.STOP);
              setConveyorSpeed(ConveyorSetpoints.STOP);
              m_pivot.set(Constants.Intake.kZeroSpeed);
            })
        .until(() -> Math.abs(m_pivotEncoder.getVelocity()) < 1.0)
        .andThen(
            this.runOnce(
                () -> {
                  m_pivot.stopMotor();
                  m_pivotEncoder.setPosition(0);
                  pivotSetpoint = 0;
                }))
        .withName("ZeroIntake");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Pivot/Position", m_pivotEncoder.getPosition());
    SmartDashboard.putNumber("Intake/Pivot/Setpoint", pivotSetpoint);
    SmartDashboard.putBoolean("Intake/Pivot/At Setpoint", pivotAtSetpoint());
  }

  public void simulationPeriodic() {
    double rollerAppliedOutput = m_roller.getAppliedOutput();
    double conveyorAppliedOutput = m_conveyor.getAppliedOutput();

    rollerSim.setVelocity(rollerAppliedOutput * 5000);
    rollerSim.setBusVoltage(12.0);
    rollerSim.setMotorCurrent(Math.abs(rollerAppliedOutput) * 40);

    conveyorSim.setVelocity(conveyorAppliedOutput * 5000);
    conveyorSim.setBusVoltage(12.0);
    conveyorSim.setMotorCurrent(Math.abs(conveyorAppliedOutput) * 40);

    SmartDashboard.putNumber("Intake/Roller/Simulated Velocity", rollerSim.getVelocity());
    SmartDashboard.putNumber("Intake/Roller/Simulated Current", rollerSim.getMotorCurrent());
    SmartDashboard.putNumber("Intake/Simulated Bus Voltage", rollerSim.getBusVoltage());
  }
}
