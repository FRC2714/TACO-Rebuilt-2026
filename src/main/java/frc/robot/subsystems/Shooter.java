package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.FeederSetpoints;
import frc.robot.Constants.ShooterConstants.FlywheelSetpoints;

public class Shooter extends SubsystemBase {

  // Initialize flywheel SPARKs. We will use MAXMotion velocity control for the flywheel, so we also
  // need to
  // initialize the closed loop controllers and encoders.
  private SparkFlex flywheelMotor =
      new SparkFlex(ShooterConstants.kFlywheelMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController flywheelController = flywheelMotor.getClosedLoopController();
  private RelativeEncoder flywheelEncoder = flywheelMotor.getEncoder();
  DCMotor flywheelGearbox = DCMotor.getNeoVortex(1);
  SparkFlexSim flywheelSim = new SparkFlexSim(flywheelMotor, flywheelGearbox);

  private SparkFlex flywheelFollowerMotor =
      new SparkFlex(ShooterConstants.kFlywheelFollowerMotorCanId, MotorType.kBrushless);
  private RelativeEncoder flywheelFollowerEncoder = flywheelFollowerMotor.getEncoder();
  DCMotor flywheelFollowerGearbox = DCMotor.getNeoVortex(1);
  SparkFlexSim flywheelFollowerSim =
      new SparkFlexSim(flywheelFollowerMotor, flywheelFollowerGearbox);

  // Initialize feeder SPARK. We will use open loop control for this so we don't need a closed loop
  // controller like above.
  private SparkFlex feederMotor =
      new SparkFlex(ShooterConstants.kFeederMotorCanId, MotorType.kBrushless);
  DCMotor feederGearbox = DCMotor.getNeoVortex(1);
  SparkFlexSim feederSim = new SparkFlexSim(feederMotor, feederGearbox);

  // Member variables for subsystem state management
  private double flywheelTargetVelocity = 0.0;

  /** Creates a new ShooterSubsystem. */
  public Shooter() {
    /*
     * Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    flywheelMotor.configure(
        Configs.Shooter.flywheelConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    flywheelFollowerMotor.configure(
        Configs.Shooter.flywheelFollowerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    feederMotor.configure(
        Configs.Shooter.feederConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Zero flywheel encoder on initialization
    flywheelEncoder.setPosition(0);

    System.out.println("---> ShooterSubsystem initialized");
  }

  private boolean isFlywheelAt(double velocity) {
    return MathUtil.isNear(
        flywheelEncoder.getVelocity(), velocity, FlywheelSetpoints.kVelocityTolerance);
  }

  /** Trigger: Is the flywheel spinning at the required velocity? */
  public final Trigger isFlywheelSpinning =
      new Trigger(() -> isFlywheelAt(5000) || flywheelEncoder.getVelocity() > 5000);

  public final Trigger isFlywheelSpinningBackwards =
      new Trigger(() -> isFlywheelAt(-5000) || flywheelEncoder.getVelocity() < -5000);

  /** Trigger: Is the flywheel stopped? */
  public final Trigger isFlywheelStopped = new Trigger(() -> isFlywheelAt(0));

  /**
   * Drive the flywheels to their set velocity. This will use MAXMotion velocity control which will
   * allow for a smooth acceleration and deceleration to the mechanism's setpoint.
   */
  private void setFlywheelVelocity(double velocity) {
    flywheelController.setSetpoint(velocity, ControlType.kMAXMotionVelocityControl);
    flywheelTargetVelocity = velocity;
  }

  /** Set the feeder motor power in the range of [-1, 1]. */
  private void setFeederPower(double power) {
    feederMotor.set(power);
  }

  /**
   * Command to run the flywheel motors. When the command is interrupted, e.g. the button is
   * released, the motors will stop.
   */
  public Command runFlywheelCommand() {
    return this.startEnd(
            () -> {
              this.setFlywheelVelocity(FlywheelSetpoints.kShootRpm);
            },
            () -> {
              this.setFlywheelVelocity(0.0);
            })
        .withName("Spinning Up Flywheel");
  }

  /**
   * Command to run the feeder and flywheel motors. When the command is interrupted, e.g. the button
   * is released, the motors will stop.
   */
  public Command runFeederCommand() {
    return this.startEnd(
            () -> {
              this.setFlywheelVelocity(FlywheelSetpoints.kShootRpm);
              this.setFeederPower(FeederSetpoints.kFeed);
            },
            () -> {
              this.setFlywheelVelocity(0.0);
              this.setFeederPower(0.0);
            })
        .withName("Feeding");
  }

  /**
   * Meta-command to operate the shooter. The Flywheel starts spinning up and when it reaches the
   * desired speed it starts the Feeder.
   */
  public Command runShooterCommand() {
    return this.startEnd(
            () -> this.setFlywheelVelocity(FlywheelSetpoints.kShootRpm),
            () -> flywheelMotor.stopMotor())
        .until(isFlywheelSpinning)
        .andThen(
            this.startEnd(
                () -> {
                  this.setFlywheelVelocity(FlywheelSetpoints.kShootRpm);
                  this.setFeederPower(FeederSetpoints.kFeed);
                },
                () -> {
                  flywheelMotor.stopMotor();
                  feederMotor.stopMotor();
                }))
        .withName("Shooting");
  }

  @Override
  public void periodic() {
    // Display subsystem values
    SmartDashboard.putNumber("Shooter | Feeder | Applied Output", feederMotor.getAppliedOutput());
    SmartDashboard.putNumber(
        "Shooter | Flywheel | Applied Output", flywheelMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shooter | Flywheel | Current", flywheelMotor.getOutputCurrent());
    SmartDashboard.putNumber(
        "Shooter | Flywheel Follower | Applied Output", flywheelFollowerMotor.getAppliedOutput());
    SmartDashboard.putNumber(
        "Shooter | Flywheel Follower | Current", flywheelFollowerMotor.getOutputCurrent());
    SmartDashboard.putNumber(
        "Shooter | Flywheel Follower | Encoder Velocity", flywheelFollowerEncoder.getVelocity());

    SmartDashboard.putNumber("Shooter | Flywheel | Target Velocity", flywheelTargetVelocity);
    SmartDashboard.putNumber("Shooter | Flywheel | Actual Velocity", flywheelEncoder.getVelocity());

    SmartDashboard.putBoolean("Is Flywheel Spinning", isFlywheelSpinning.getAsBoolean());
    SmartDashboard.putBoolean("Is Flywheel Stopped", isFlywheelStopped.getAsBoolean());
  }

  @Override
  public void simulationPeriodic() {
    // Simulate flywheel behavior using SparkFlexSim APIs (setVelocity / setMotorCurrent)
    double flywheelApplied = flywheelMotor.getAppliedOutput();
    // crude mapping: applied output -> RPM/current
    flywheelSim.setVelocity(flywheelApplied * 6000.0);
    flywheelSim.setBusVoltage(12.0);
    flywheelSim.setMotorCurrent(Math.abs(flywheelApplied) * 40.0);

    double followerApplied = flywheelFollowerMotor.getAppliedOutput();
    flywheelFollowerSim.setVelocity(followerApplied * 6000.0);
    flywheelFollowerSim.setBusVoltage(12.0);
    flywheelFollowerSim.setMotorCurrent(Math.abs(followerApplied) * 40.0);

    double feederApplied = feederMotor.getAppliedOutput();
    feederSim.setVelocity(feederApplied * 4000.0);
    feederSim.setBusVoltage(12.0);
    feederSim.setMotorCurrent(Math.abs(feederApplied) * 20.0);

    // Display simulated values
    SmartDashboard.putNumber("Shooter | Flywheel | Simulated Velocity", flywheelSim.getVelocity());
    SmartDashboard.putNumber(
        "Shooter | Flywheel | Simulated Current", flywheelSim.getMotorCurrent());
    SmartDashboard.putNumber(
        "Shooter | Flywheel Follower | Simulated Velocity", flywheelFollowerSim.getVelocity());
    SmartDashboard.putNumber(
        "Shooter | Flywheel Follower | Simulated Current", flywheelFollowerSim.getMotorCurrent());
    SmartDashboard.putNumber("Shooter | Feeder | Simulated Velocity", feederSim.getVelocity());
    SmartDashboard.putNumber("Shooter | Feeder | Simulated Current", feederSim.getMotorCurrent());
  }
}
