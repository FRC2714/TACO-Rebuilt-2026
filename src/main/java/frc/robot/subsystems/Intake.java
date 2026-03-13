package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private SparkFlex m_pivot = new SparkFlex(Constants.Intake.kPivotCanId, MotorType.kBrushless);

  private AbsoluteEncoder m_pivotEncoder = m_pivot.getAbsoluteEncoder();

  private SparkClosedLoopController m_pivotController = m_pivot.getClosedLoopController();

  private SparkFlex m_roller = new SparkFlex(Constants.Intake.kRollerCanId, MotorType.kBrushless);
  private SparkFlex m_conveyor =
      new SparkFlex(Constants.Intake.kConveyorCanId, MotorType.kBrushless);

  private SparkSim rollerSim;
  private SparkSim conveyorSim;

  private double pivotSpeed = Constants.Intake.PivotSetpoints.kStow;

  private enum PivotSetpoints {
    STOW,
    INTAKE,
    EXTAKE
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
    // Constructor code here
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
      default:
        return;
    }
    pivotSpeed = setSpeed;
    m_pivot.set(setSpeed);
  }

  public Command intakeCommand() {
    return this.startEnd(
            () -> {
              setPivot(PivotSetpoints.INTAKE);
              setRollerSpeed(RollerSetpoints.INTAKE);
              setConveyorSpeed(ConveyorSetpoints.INTAKE);
            },
            () -> {
              m_roller.stopMotor();
              m_conveyor.stopMotor();
            })
        .withName("Intaking");
  }

  public Command conveyorCommand() {
    return this.startEnd(
            () -> {
              setConveyorSpeed(ConveyorSetpoints.INTAKE);
            },
            () -> {
              m_conveyor.stopMotor();
            })
        .withName("Conveyoring");
  }

  public Command extakeCommand() {
    return this.run(
        () -> {
          setPivot(PivotSetpoints.EXTAKE);
          setRollerSpeed(RollerSetpoints.EXTAKE);
          setConveyorSpeed(ConveyorSetpoints.EXTAKE);
        });
  }

  public Command stowCommand() {
    return this.run(
        () -> {
          setRollerSpeed(RollerSetpoints.STOP);
          setPivot(PivotSetpoints.STOW);
          setConveyorSpeed(ConveyorSetpoints.STOP);
        });
  }

  // public boolean atSetpoint() {
  // return Math.abs(m_pivotEncoder.getPosition() - pivotSetpoint)
  // <= Constants.Intake.kPivotThreshold;
  // }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Intake/Pivot/Position",
    // m_pivotEncoder.getPosition());
    SmartDashboard.putNumber("Intake/Pivot/Setpoint", pivotSpeed);
    // SmartDashboard.putBoolean("Intake/Pivot/At Setpoint?", atSetpoint());
  }

  public void simulationPeriodic() {
    // Example: simulate velocity based on applied output
    double rollerAppliedOutput = m_roller.getAppliedOutput();
    double conveyorAppliedOutput = m_conveyor.getAppliedOutput();

    rollerSim.setVelocity(rollerAppliedOutput * 5000); // fake RPM model
    rollerSim.setBusVoltage(12.0);
    rollerSim.setMotorCurrent(Math.abs(rollerAppliedOutput) * 40);

    conveyorSim.setVelocity(rollerAppliedOutput * 5000); // fake RPM model
    conveyorSim.setBusVoltage(12.0);
    conveyorSim.setMotorCurrent(Math.abs(rollerAppliedOutput) * 40);

    SmartDashboard.putNumber("Simulated Roller Velocity: ", rollerSim.getVelocity());
    SmartDashboard.putNumber("Simulated Roller Current: ", rollerSim.getMotorCurrent());
    SmartDashboard.putNumber("Simulated Bus Voltage: ", rollerSim.getBusVoltage());
  }
}
