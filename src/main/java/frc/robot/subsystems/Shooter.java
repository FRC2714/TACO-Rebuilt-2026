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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.FeederSetpoints;
import frc.robot.Constants.ShooterConstants.FlywheelSetpoints;

public class Shooter extends SubsystemBase {

  // Initialize flywheel SPARKs. We will use MAXMotion velocity control for the
  // flywheel, so we also
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

  // Initialize feeder SPARK. We will use open loop control for this so we don't
  // need a closed loop
  // controller like above.
  private SparkFlex feederMotor =
      new SparkFlex(ShooterConstants.kFeederMotorCanId, MotorType.kBrushless);
  DCMotor feederGearbox = DCMotor.getNeoVortex(1);
  SparkFlexSim feederSim = new SparkFlexSim(feederMotor, feederGearbox);

  // Member variables for subsystem state management
  private double flywheelTargetVelocity = 0.0;
  private double calculatedRpm = FlywheelSetpoints.kShootRpm;
  private double calculatedHeadingDeg = 0.0;
  private boolean isShooting = false;

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

  /** Trigger: Is the flywheel spinning at the calculated target velocity? */
  public final Trigger isFlywheelSpinning =
      new Trigger(
          () ->
              Math.abs(flywheelEncoder.getVelocity() - calculatedRpm)
                      < FlywheelSetpoints.kVelocityTolerance
                  || flywheelEncoder.getVelocity() > calculatedRpm);

  public final Trigger isFlywheelSpinningBackwards =
      new Trigger(() -> flywheelEncoder.getVelocity() < -calculatedRpm);

  public record ShooterParams(double rpm, double timeOfFlight) {
    public static ShooterParams interpolate(ShooterParams a, ShooterParams b, double t) {
      double rpm = a.rpm + (b.rpm - a.rpm) * t;
      double timeOfFlight = a.timeOfFlight + (b.timeOfFlight - a.timeOfFlight) * t;
      return new ShooterParams(rpm, timeOfFlight);
    }
  }

  /** Trigger: Is the flywheel stopped? */
  public final Trigger isFlywheelStopped = new Trigger(() -> isFlywheelAt(0));

  /**
   * Drive the flywheels to their set velocity. This will use MAXMotion velocity control which will
   * allow for a smooth acceleration and deceleration to the mechanism's setpoint.
   */
  private void setFlywheelVelocity(double velocity) {
    flywheelController.setSetpoint(velocity, ControlType.kVelocity);
    flywheelTargetVelocity = velocity;
  }

  /** Set the feeder motor power in the range of [-1, 1]. */
  private void setFeederPower(double power) {
    feederMotor.set(power);
  }

  private static final InterpolatingTreeMap<Double, ShooterParams> shooterMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShooterParams::interpolate);

  static {
    // TODO: Tune these values on the actual robot
    shooterMap.put(1.626, new ShooterParams(1850, 0.89));
    shooterMap.put(2.68, new ShooterParams(2200, 1.09));
    shooterMap.put(2.9464, new ShooterParams(2500, 1.27));
    shooterMap.put(3.17, new ShooterParams(2400, 1.2));
    shooterMap.put(3.4, new ShooterParams(2600, 1.2));
    shooterMap.put(4.2, new ShooterParams(3200, 1.3));
    shooterMap.put(4.5, new ShooterParams(3900, 1.3));
    shooterMap.put(4.65, new ShooterParams(3800, 1.4));
    shooterMap.put(5.2, new ShooterParams(3900, 1.4));
  }

  public void calculate(
      Translation2d robotPosition,
      Rotation2d robotHeading,
      Translation2d robotVelocity,
      Translation2d goalPosition,
      double latencyCompensation) {

    Translation2d futurePos = robotPosition.plus(robotVelocity.times(latencyCompensation));
    Translation2d relativePosition = goalPosition.minus(futurePos);
    Translation2d relativeVelocity = robotVelocity.times(-1);

    // ShooterParams rawParams = shooterMap.get(relativePosition.getNorm());
    // this.rawFlywheelTarget = rawParams.rpm; Not needed?
    double timeOfFlight = 0.0;
    Translation2d adjustedRelativePosition = relativePosition;

    final int MAX_ITERATIONS = 10;
    final double CONVERGENCE_THRESHOLD = 0.001;

    for (int i = 0; i < MAX_ITERATIONS; i++) {
      double distance = adjustedRelativePosition.getNorm();
      ShooterParams params = shooterMap.get(distance);
      double newTimeOfFlight = params.timeOfFlight;

      // Check convergence before updating so we exit with the stable TOF value
      if (Math.abs(newTimeOfFlight - timeOfFlight) < CONVERGENCE_THRESHOLD) {
        timeOfFlight = newTimeOfFlight;
        break;
      }

      timeOfFlight = newTimeOfFlight;

      // Predict where the target will be (relative to the robot) when the
      // gamepiece arrives: shift the relative position by relative velocity * TOF
      adjustedRelativePosition = relativePosition.plus(relativeVelocity.times(timeOfFlight));
    }

    double adjustedDistance = adjustedRelativePosition.getNorm();
    ShooterParams adjustedParams = shooterMap.get(adjustedDistance);
    // Project robot velocity onto the away-from-hub direction
    // Positive = moving away (needs more RPM), negative = moving toward (needs less)
    Translation2d awayFromHub = relativePosition.div(relativePosition.getNorm()).times(-1);
    double radialSpeed = robotVelocity.getX() * awayFromHub.getX()
        + robotVelocity.getY() * awayFromHub.getY();
    double lateralSpeed = Math.abs(robotVelocity.getNorm() * robotVelocity.getNorm()
        - radialSpeed * radialSpeed);
    lateralSpeed = Math.sqrt(Math.max(0, lateralSpeed));
    double velocityRpmBoost = radialSpeed * ShooterConstants.kRpmPerMpsRadial
        + lateralSpeed * ShooterConstants.kRpmPerMpsLateral;
    calculatedRpm = (adjustedParams.rpm + velocityRpmBoost) * ShooterConstants.kRpmScaleFactor;
    calculatedHeadingDeg = adjustedRelativePosition.getAngle().getDegrees();
  }

  public double getCalculatedRpm() {
    return calculatedRpm;
  }

  public double getCalculatedHeadingDeg() {
    return calculatedHeadingDeg;
  }

  public void setIsShooting(boolean shooting) {
    this.isShooting = shooting;
  }

  private Debouncer flywheelDebouncer =
      new Debouncer(ShooterConstants.kFlywheelDebounceTimeSeconds, DebounceType.kFalling);

  public boolean flywheelAtSetpoint() {
    boolean atSetpoint =
        Math.abs(flywheelEncoder.getVelocity() - flywheelTargetVelocity)
            < 300; // Should eb in constants? maybe needs to be tuned.
    return flywheelDebouncer.calculate(atSetpoint);
  }

  /**
   * Command to run the flywheel motors. When the command is interrupted, e.g. the button is
   * released, the motors will stop.
   */
  public Command runFlywheelCommand() {
    return this.run(() -> this.setFlywheelVelocity(calculatedRpm))
        .finallyDo(() -> this.setFlywheelVelocity(0.0))
        .withName("Spinning Up Flywheel");
  }

  /**
   * Command to run the feeder and flywheel motors. When the command is interrupted, e.g. the button
   * is released, the motors will stop.
   */
  public Command runFeederCommand() {
    return this.run(() -> {
              this.setFlywheelVelocity(calculatedRpm);
              this.setFeederPower(FeederSetpoints.kFeed);
            })
        .finallyDo(() -> {
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
    Command spinUntilUp =
        this.run(() -> this.setFlywheelVelocity(calculatedRpm))
            .until(isFlywheelSpinning)
            .withName("SpinUntilUp");

    Command feederPhase =
        this.run(() -> {
                  this.setFlywheelVelocity(calculatedRpm);
                  this.setFeederPower(FeederSetpoints.kFeed);
                })
            .finallyDo(() -> {
                  this.setFlywheelVelocity(0.0);
                  this.setFeederPower(0.0);
                })
            .withName("FeederPhase");

    return spinUntilUp.andThen(feederPhase).withName("Shooting");
  }

  public Command runShooter() {
    return this.run(() -> this.setFlywheelVelocity(calculatedRpm))
        .until(isFlywheelSpinning)
        .andThen(
            this.run(
                () -> {
                  this.setFlywheelVelocity(calculatedRpm);
                  this.setFeederPower(FeederSetpoints.kFeed);
                }));
  }

  /** Command to run only the feeder motor at the given power. No flywheel. */
  public Command runFeederOnlyCommand(double power) {
    return this.startEnd(() -> this.setFeederPower(power), () -> this.setFeederPower(0.0))
        .withName("Feeder Only");
  }

  public void stopAll() {
    flywheelMotor.stopMotor();
    feederMotor.stopMotor();
  }

  public Command stopShooter() {
    return new InstantCommand(this::stopAll);
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
    SmartDashboard.putNumber("Shooter | Calculated RPM", calculatedRpm);
  }

  @Override
  public void simulationPeriodic() {
    // Simulate flywheel behavior using SparkFlexSim APIs (setVelocity /
    // setMotorCurrent)
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
