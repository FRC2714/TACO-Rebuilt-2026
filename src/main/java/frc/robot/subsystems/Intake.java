package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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
  private SparkFlex m_roller = new SparkFlex(Constants.Intake.kRollerCanId, MotorType.kBrushless);
  private SparkFlex m_conveyor =
      new SparkFlex(Constants.Intake.kConveyorCanId, MotorType.kBrushless);

  private SparkSim rollerSim;
  private SparkSim conveyorSim;

  private double pivotSpeed = Constants.Intake.PivotSetpoints.kStow;

  private enum PivotSetpoints {
    STOW,
    INTAKE,
    EXTAKE,
    AGITATE
  }

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

  private void setPivot(PivotSetpoints setpoint) {
    double setSpeed = 0;
    switch (setpoint) {
      case STOW:
        setSpeed = Constants.Intake.PivotSetpoints.kStow;
        break;
      case INTAKE:
        setSpeed = Constants.Intake.PivotSetpoints.kIntake;
        break;
      case EXTAKE:
        setSpeed = Constants.Intake.PivotSetpoints.kExtake;
        break;
      case AGITATE:
        setSpeed = Constants.Intake.PivotSetpoints.kAgitate;
        break;
      default:
        return;
    }
    pivotSpeed = setSpeed;
    m_pivot.set(setSpeed);
  }

  /** Stow pivot and stop all motors. */
  public void stopAll() {
    setPivot(PivotSetpoints.STOW);
    setRollerSpeed(RollerSetpoints.STOP);
    setConveyorSpeed(ConveyorSetpoints.STOP);
  }

  public Command intakeCommand() {
    return this.startEnd(
            () -> {
              setPivot(PivotSetpoints.INTAKE);
              setRollerSpeed(RollerSetpoints.INTAKE);
              setConveyorSpeed(ConveyorSetpoints.INTAKE);
            },
            () -> {
              setRollerSpeed(RollerSetpoints.STOP);
              setConveyorSpeed(ConveyorSetpoints.STOP);
            })
        .withName("Intaking");
  }

  public Command conveyorCommand() {
    return this.startEnd(
            () -> setConveyorSpeed(ConveyorSetpoints.INTAKE),
            () -> setConveyorSpeed(ConveyorSetpoints.STOP))
        .withName("Conveyoring");
  }

  public Command extakeCommand() {
    return this.startEnd(
        () -> {
          setPivot(PivotSetpoints.EXTAKE);
          setRollerSpeed(RollerSetpoints.EXTAKE);
          setConveyorSpeed(ConveyorSetpoints.EXTAKE);
        },
        () -> {
          setRollerSpeed(RollerSetpoints.STOP);
          setConveyorSpeed(ConveyorSetpoints.STOP);
        });
  }

  /** Single agitation cycle: stow with rollers intaking, then deploy. */
  public Command agitateCommand() {
    double stowTime = Constants.Intake.AgitatorConstants.kStowDuration;
    double deployTime = Constants.Intake.AgitatorConstants.kDeployDuration;

    return this.runOnce(
            () -> {
              setPivot(PivotSetpoints.STOW);
              setRollerSpeed(RollerSetpoints.INTAKE);
            })
        .andThen(Commands.waitSeconds(stowTime))
        .andThen(
            this.runOnce(
                () -> {
                  setPivot(PivotSetpoints.AGITATE);
                  setRollerSpeed(RollerSetpoints.STOP);
                }))
        .andThen(Commands.waitSeconds(deployTime))
        .withName("Agitating");
  }

  /** Pivot-only agitation for use during shooting. Does not claim Intake subsystem. */
  public Command shootingAgitateCommand() {
    double stowTime = Constants.Intake.AgitatorConstants.kStowDuration;
    double deployTime = Constants.Intake.AgitatorConstants.kDeployDuration;
    int count = Constants.Intake.AgitatorConstants.kAgitationCount;

    Command agitation = Commands.none();
    for (int i = 0; i < count; i++) {
      agitation =
          agitation
              .andThen(Commands.runOnce(() -> setPivot(PivotSetpoints.STOW)))
              .andThen(Commands.waitSeconds(stowTime))
              .andThen(Commands.runOnce(() -> setPivot(PivotSetpoints.AGITATE)))
              .andThen(Commands.waitSeconds(deployTime));
    }

    return Commands.waitSeconds(Constants.Intake.AgitatorConstants.kDelayBeforeAgitating)
        .andThen(agitation)
        .withName("ShootingAgitate");
  }

  public Command stowCommand() {
    return this.run(this::stopAll);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Pivot/Setpoint", pivotSpeed);
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
