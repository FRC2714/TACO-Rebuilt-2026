package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.Intake.PivotSetpoints;
import frc.robot.Constants.Intake.RollerSetpoints;

public class Intake extends SubsystemBase {
  //   private SparkFlex m_pivot = new SparkFlex(Constants.Intake.kPivotCanId,
  // MotorType.kBrushless);

  //   private AbsoluteEncoder m_pivotEncoder = m_pivot.getAbsoluteEncoder();

  //   private SparkClosedLoopController m_pivotController = m_pivot.getClosedLoopController();

  private SparkFlex m_roller = new SparkFlex(Constants.Intake.kRollerCanId, MotorType.kBrushless);

  private SparkSim rollerSim;
  private SparkFlex motor;
  private SparkSim motorSim;

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

  public Intake() {
    // Constructor code here
    // m_pivot.configure(
    //     Configs.Intake.pivotConfig, ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);
    m_roller.configure(
        Configs.Intake.rollerConfig,
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

  private void setPivot(PivotSetpoints setpoint) {
    double targetPosition = 0;
    switch (setpoint) {
      case STOW:
        targetPosition = Constants.Intake.PivotSetpoints.kStow;
        break;
      case INTAKE:
        targetPosition = Constants.Intake.PivotSetpoints.kIntake;
        break;
      case EXTAKE:
        targetPosition = Constants.Intake.PivotSetpoints.kExtake;
        break;
      default:
        return;
    }
    // m_pivotController.setSetpoint(targetPosition, ControlType.kPosition);
  }

  public Command intakeCommand() {
    return this.run(
        () -> {
          //  setPivot(PivotSetpoints.INTAKE);
          setRollerSpeed(RollerSetpoints.INTAKE);
        });
  }

  public Command extakeCommand() {
    return this.run(
        () -> {
          setRollerSpeed(RollerSetpoints.EXTAKE);
          //  setPivot(PivotSetpoints.EXTAKE);
        });
  }

 
  public Command stowCommand() {
    return this.run(
        () -> {
          setRollerSpeed(RollerSetpoints.STOP);
          //  setPivot(PivotSetpoints.STOW);
        });
  }

  public void simulationPeriodic() {
    // Example: simulate velocity based on applied output
    double appliedOutput = m_roller.getAppliedOutput();

    rollerSim.setVelocity(appliedOutput * 5000); // fake RPM model
    rollerSim.setBusVoltage(12.0);
    rollerSim.setMotorCurrent(Math.abs(appliedOutput) * 40);
  }
}
