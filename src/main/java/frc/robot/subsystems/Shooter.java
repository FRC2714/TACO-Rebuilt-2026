package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
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

  private SparkFlex feederMotor =
      new SparkFlex(ShooterConstants.kFeederMotorCanId, MotorType.kBrushless);
  DCMotor feederGearbox = DCMotor.getNeoVortex(1);
  SparkFlexSim feederSim = new SparkFlexSim(feederMotor, feederGearbox);

  private double flywheelTargetVelocity = 0.0;
  private double calculatedRpm = FlywheelSetpoints.kShootRpm;
  private double calculatedHeadingDeg = 0.0;
  private boolean autoPreSpin = false;

  public Shooter() {
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

    flywheelEncoder.setPosition(0);
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

  /** Trigger: Is the flywheel stopped? */
  public final Trigger isFlywheelStopped =
      new Trigger(
          () -> Math.abs(flywheelEncoder.getVelocity()) < FlywheelSetpoints.kVelocityTolerance);

  public record ShooterParams(double rpm, double timeOfFlight) {
    public static ShooterParams interpolate(ShooterParams a, ShooterParams b, double t) {
      double rpm = a.rpm + (b.rpm - a.rpm) * t;
      double timeOfFlight = a.timeOfFlight + (b.timeOfFlight - a.timeOfFlight) * t;
      return new ShooterParams(rpm, timeOfFlight);
    }
  }

  private void setFlywheelVelocity(double velocity) {
    flywheelController.setSetpoint(velocity, ControlType.kVelocity);
    flywheelTargetVelocity = velocity;
  }

  private void setFeederPower(double power) {
    feederMotor.set(power);
  }

  private static final InterpolatingTreeMap<Double, ShooterParams> shooterMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShooterParams::interpolate);

  static {
    shooterMap.put(1.626, new ShooterParams(1850, 0.89));
    shooterMap.put(2.68, new ShooterParams(2375, 1.09));
    shooterMap.put(2.9464, new ShooterParams(2500, 1.27));
    shooterMap.put(3.17, new ShooterParams(2620, 1.2));
    shooterMap.put(3.4, new ShooterParams(2675, 1.2));
    shooterMap.put(3.8, new ShooterParams(2775, 1.2));
    shooterMap.put(4.2, new ShooterParams(3250, 1.3));
    shooterMap.put(4.5, new ShooterParams(3450, 1.3));
    shooterMap.put(4.65, new ShooterParams(3775, 1.4));
    shooterMap.put(5.2, new ShooterParams(3800, 1.4));
    shooterMap.put(6.0, new ShooterParams(4300, 1.5));
    shooterMap.put(7.0, new ShooterParams(4650, 1.6));
    shooterMap.put(8.0, new ShooterParams(5200, 1.7));
    shooterMap.put(9.0, new ShooterParams(5900, 1.8));
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

    double timeOfFlight = 0.0;
    Translation2d adjustedRelativePosition = relativePosition;

    final int MAX_ITERATIONS = 10;
    final double CONVERGENCE_THRESHOLD = 0.001;

    for (int i = 0; i < MAX_ITERATIONS; i++) {
      double distance = adjustedRelativePosition.getNorm();
      ShooterParams params = shooterMap.get(distance);
      double newTimeOfFlight = params.timeOfFlight;

      if (Math.abs(newTimeOfFlight - timeOfFlight) < CONVERGENCE_THRESHOLD) {
        timeOfFlight = newTimeOfFlight;
        break;
      }

      timeOfFlight = newTimeOfFlight;
      adjustedRelativePosition = relativePosition.plus(relativeVelocity.times(timeOfFlight));
    }

    double adjustedDistance = adjustedRelativePosition.getNorm();
    ShooterParams adjustedParams = shooterMap.get(adjustedDistance);

    Translation2d awayFromHub = relativePosition.div(relativePosition.getNorm()).times(-1);
    double radialSpeed =
        robotVelocity.getX() * awayFromHub.getX() + robotVelocity.getY() * awayFromHub.getY();
    double lateralSpeedSq =
        robotVelocity.getNorm() * robotVelocity.getNorm() - radialSpeed * radialSpeed;
    double lateralSpeed = Math.sqrt(Math.max(0, lateralSpeedSq));
    double velocityRpmBoost =
        radialSpeed * ShooterConstants.kRpmPerMpsRadial
            + lateralSpeed * ShooterConstants.kRpmPerMpsLateral;

    calculatedRpm = (adjustedParams.rpm + velocityRpmBoost) * ShooterConstants.kRpmScaleFactor;

    // Compute signed lateral velocity for heading compensation
    // Cross product of velocity x toHub gives signed lateral component (sign matches needed lead)
    Translation2d toHub = relativePosition.div(relativePosition.getNorm());
    double signedLateralSpeed =
        robotVelocity.getX() * toHub.getY() - robotVelocity.getY() * toHub.getX();
    double lateralHeadingOffset = signedLateralSpeed * ShooterConstants.kHeadingPerMpsLateral;

    calculatedHeadingDeg = adjustedRelativePosition.getAngle().getDegrees() + lateralHeadingOffset;
  }

  public double getCalculatedRpm() {
    return calculatedRpm;
  }

  public double getCalculatedHeadingDeg() {
    return calculatedHeadingDeg;
  }

  public void setAutoPreSpin(boolean active) {
    this.autoPreSpin = active;
  }

  /** Pre-spin the flywheel to calculatedRpm. Keeps running until interrupted by another command. */
  public Command preSpinCommand() {
    return this.run(() -> this.setFlywheelVelocity(calculatedRpm)).withName("PreSpin");
  }

  /** Spin up flywheel, then run feeder once at speed. Uses calculatedRpm (updates every cycle). */
  public Command runShooterCommand() {
    // Reverse feeder briefly to normalize ball positions while flywheel spins up.
    // Ends early if flywheel reaches speed before 0.5s.
    Command unjam =
        this.run(
                () -> {
                  this.setFlywheelVelocity(calculatedRpm);
                  this.setFeederPower(-FeederSetpoints.kFeed);
                })
            .until(isFlywheelSpinning)
            .withTimeout(ShooterConstants.kUnjamDuration)
            .withName("Unjam");

    // Continue spinning up (feeder stopped) if flywheel isn't ready yet after unjam
    Command spinUntilUp =
        this.run(
                () -> {
                  this.setFlywheelVelocity(calculatedRpm);
                  this.setFeederPower(0);
                })
            .until(isFlywheelSpinning)
            .withName("SpinUntilUp");

    Command feederPhase =
        this.run(
                () -> {
                  this.setFlywheelVelocity(calculatedRpm);
                  this.setFeederPower(FeederSetpoints.kFeed);
                })
            .finallyDo(this::stopAll)
            .withName("FeederPhase");

    return unjam.andThen(spinUntilUp).andThen(feederPhase).withName("Shooting");
  }

  public Trigger getIsFlywheelSpinning() {
    return isFlywheelSpinning;
  }

  /** Spin up flywheel using calculatedRpm (no auto-align override), then run feeder. */
  public Command runPassCommand() {
    Command spinUntilUp =
        this.run(() -> this.setFlywheelVelocity(calculatedRpm))
            .until(isFlywheelSpinning)
            .withName("SpinUntilUp");

    Command feederPhase =
        this.run(
                () -> {
                  this.setFlywheelVelocity(FlywheelSetpoints.kPassRpm);
                  this.setFeederPower(FeederSetpoints.kFeed);
                })
            .finallyDo(this::stopAll)
            .withName("FeederPhase");

    return spinUntilUp.andThen(feederPhase).withName("Passing");
  }

  /** Run only the feeder motor at the given power. No flywheel. */
  public Command runFeederOnlyCommand(double power) {
    return this.startEnd(() -> this.setFeederPower(power), () -> this.setFeederPower(0.0))
        .withName("Feeder Only");
  }

  public void stopAll() {
    flywheelMotor.stopMotor();
    feederMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // Auto pre-spin: keep flywheel at speed when hub is active + in alliance zone
    // Only when no command (like shoot) is actively controlling the shooter
    if (getCurrentCommand() == null) {
      if (autoPreSpin) {
        setFlywheelVelocity(calculatedRpm);
      } else {
        setFlywheelVelocity(0);
      }
    }

    SmartDashboard.putNumber("Shooter/Feeder/Applied Output", feederMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shooter/Flywheel/Applied Output", flywheelMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shooter/Flywheel/Current", flywheelMotor.getOutputCurrent());
    SmartDashboard.putNumber(
        "Shooter/Flywheel Follower/Applied Output", flywheelFollowerMotor.getAppliedOutput());
    SmartDashboard.putNumber(
        "Shooter/Flywheel Follower/Current", flywheelFollowerMotor.getOutputCurrent());
    SmartDashboard.putNumber(
        "Shooter/Flywheel Follower/Encoder Velocity", flywheelFollowerEncoder.getVelocity());

    SmartDashboard.putNumber("Shooter/Flywheel/Target Velocity", flywheelTargetVelocity);
    SmartDashboard.putNumber("Shooter/Flywheel/Actual Velocity", flywheelEncoder.getVelocity());

    SmartDashboard.putBoolean("Is Flywheel Spinning", isFlywheelSpinning.getAsBoolean());
    SmartDashboard.putBoolean("Is Flywheel Stopped", isFlywheelStopped.getAsBoolean());
    SmartDashboard.putNumber("Shooter/Calculated RPM", calculatedRpm);
  }

  @Override
  public void simulationPeriodic() {
    double flywheelApplied = flywheelMotor.getAppliedOutput();
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

    SmartDashboard.putNumber("Shooter/Flywheel/Simulated Velocity", flywheelSim.getVelocity());
    SmartDashboard.putNumber("Shooter/Flywheel/Simulated Current", flywheelSim.getMotorCurrent());
    SmartDashboard.putNumber(
        "Shooter/Flywheel Follower/Simulated Velocity", flywheelFollowerSim.getVelocity());
    SmartDashboard.putNumber(
        "Shooter/Flywheel Follower/Simulated Current", flywheelFollowerSim.getMotorCurrent());
    SmartDashboard.putNumber("Shooter/Feeder/Simulated Velocity", feederSim.getVelocity());
    SmartDashboard.putNumber("Shooter/Feeder/Simulated Current", feederSim.getMotorCurrent());
  }
}
