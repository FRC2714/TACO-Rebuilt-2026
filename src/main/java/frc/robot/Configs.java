package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants.ModuleConstants;

public final class Configs {

  public static final class MAXSwerveModule {
    public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
    public static final SparkFlexConfig turningConfig = new SparkFlexConfig();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor =
          ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;
      double nominalVoltage = 12.0;
      double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

      drivingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(70);
      drivingConfig
          .encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(0.04, 0, 0)
          .outputRange(-1, 1)
          .feedForward
          .kV(drivingVelocityFeedForward);

      turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(70);

      turningConfig
          .absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(true)
          .positionConversionFactor(turningFactor) // radians
          .velocityConversionFactor(turningFactor / 60.0) // radians per second
          .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder);

      turningConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(1, 0, 0)
          .outputRange(-1, 1)
          // Enable PID wrap around for the turning motor. This will allow the PID
          // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
          // to 10 degrees will go through 0 rather than the other direction which is a
          // longer route.
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
    }
  }

  public static final class Intake {
    public static final SparkFlexConfig rollerConfig = new SparkFlexConfig();
    public static final SparkFlexConfig pivotConfig = new SparkFlexConfig();
    public static final SparkFlexConfig conveyorConfig = new SparkFlexConfig();

    static {
      // Configure basic setting of the pivot motor
      pivotConfig
          .smartCurrentLimit(40)
          .idleMode(IdleMode.kBrake)
          .inverted(true)
          .voltageCompensation(12);
      // Use built-in relative encoder (7168 counts/rev on Neo Vortex)
      // Convert to degrees of output shaft
      pivotConfig
          .encoder
          .positionConversionFactor(360.0 / Constants.Intake.kPivotReduction)
          .velocityConversionFactor(360.0 / Constants.Intake.kPivotReduction / 60.0);
      pivotConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .p(Constants.Intake.kP)
          .d(Constants.Intake.kD)
          .outputRange(-1, 1);

      // Configure basic settings of the intake motor
      rollerConfig
          .inverted(false)
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(80)
          .voltageCompensation(12);
      conveyorConfig
          .inverted(false)
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(80)
          .voltageCompensation(12);
    }
  }

  public static final class Shooter {
    public static final SparkFlexConfig flywheelConfig = new SparkFlexConfig();
    public static final SparkFlexConfig flywheelFollowerConfig = new SparkFlexConfig();
    public static final SparkFlexConfig feederConfig = new SparkFlexConfig();

    static {
      // Configure basic setting of the flywheel motors
      flywheelConfig
          .inverted(true)
          .idleMode(IdleMode.kCoast)
          .closedLoopRampRate(0.3)
          .openLoopRampRate(1.0)
          .smartCurrentLimit(80);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      flywheelConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.00055)
          .outputRange(-1, 1);

      flywheelConfig.encoder.quadratureMeasurementPeriod(10).quadratureAverageDepth(4);

      //   flywheelConfig
      //       .closedLoop
      //       .maxMotion
      //       // Set MAXMotion parameters for MAXMotion Velocity control
      //       .cruiseVelocity(2000)
      //       .maxAcceleration(4000)
      //       .allowedProfileError(1);

      // Constants.NeoMotorConstants.kVortexKv is in rpm/V. feedforward.kV is in V/rpm sort we take
      // the reciprocol.
      flywheelConfig.closedLoop.feedForward.kV(.00182);

      //   flywheelConfig.encoder.velocityConversionFactor(34./46.);

      // Configure the follower flywheel motor to follow the main flywheel motor
      flywheelFollowerConfig
          .apply(flywheelConfig)
          .follow(Constants.ShooterConstants.kFlywheelMotorCanId, true);

      // Configure basic setting of the feeder motor
      feederConfig
          .inverted(true)
          .idleMode(IdleMode.kCoast)
          .openLoopRampRate(1.0)
          .smartCurrentLimit(60);
    }
  }
}
