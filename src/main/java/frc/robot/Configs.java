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

      drivingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
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

      turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);

      turningConfig
          .absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(true)
          .positionConversionFactor(turningFactor) // radians
          .velocityConversionFactor(turningFactor / 60.0) // radians per second
          // This applies to REV Through Bore Encoder V2 (use REV_ThroughBoreEncoder for V1):
          .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

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

    static {
      // Configure basic setting of the arm motor
      pivotConfig
          .smartCurrentLimit(40)
          .idleMode(IdleMode.kBrake)
          .inverted(true)
          .voltageCompensation(12);
      pivotConfig
          .absoluteEncoder
          .positionConversionFactor(360 / Constants.Intake.kPivotReduction)
          .inverted(false)
          .zeroCentered(false); // tune later
      pivotConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // Set PID values for position control. We don't need to pass a closed
          // loop slot, as it will default to slot 0.
          .p(Constants.Intake.kP)
          .d(0)
          .outputRange(-1, 1)
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, 360)
          .maxMotion
          .maxVelocity(4200 * 360)
          .maxAcceleration(6000 * 360)
          .allowedClosedLoopError(0.5);

      // Configure basic settings of the intake motor
      rollerConfig
          .inverted(false)
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(40)
          .voltageCompensation(12);
    }
  }
}
