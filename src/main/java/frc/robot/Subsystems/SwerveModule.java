package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */

  private final SparkMax driveMotor;
  private final SparkMax rotationMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder rotationEncoder;

  private final CANcoder absoluteEncoder;

  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffset;

  private final LoggedTunableNumber kPRotation = new LoggedTunableNumber("Tuning/kPRotation", 0.5);

  public SwerveModule(int driveMotorID, int rotationMotorID, int absoluteEncoderID, boolean absoluteEncoderReversed, double absoluteEncoderOffset, boolean driveInverted, boolean rotationInverted) {
    //motors
    this.driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
    this.rotationMotor = new SparkMax(rotationMotorID, MotorType.kBrushless);

    //Encoders
    this.driveEncoder = driveMotor.getEncoder();
    this.rotationEncoder = rotationMotor.getEncoder();
    this.absoluteEncoder = new CANcoder(absoluteEncoderID);

    //drive motor config
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig.inverted(driveInverted);
    driveConfig.idleMode(IdleMode.kBrake);
    driveConfig.encoder.positionConversionFactor((Math.PI * Constants.wheelDiameterMeters)/Constants.driveMotorGearRatio);
    driveConfig.encoder.velocityConversionFactor((Math.PI * Constants.wheelDiameterMeters)/(60 * Constants.driveMotorGearRatio));
    this.driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //rotation motor config
    SparkMaxConfig rotationConfig = new SparkMaxConfig();
    rotationConfig.inverted(rotationInverted);
    rotationConfig.idleMode(IdleMode.kBrake);
    rotationConfig.closedLoop.pid(Constants.kPRotation,Constants.kIRotation, Constants.kDRotation);
    rotationConfig.closedLoop.positionWrappingEnabled(true);
    rotationConfig.closedLoop.positionWrappingInputRange(-2*Math.PI, 2*Math.PI);
    rotationConfig.encoder.positionConversionFactor(Constants.rotationsToRad/Constants.rotationMotorGearRatio);
    this.rotationMotor.configure(rotationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.absoluteEncoderReversed = absoluteEncoderReversed;
    this.absoluteEncoderOffset = absoluteEncoderOffset;
    
    //need to configure PID stuff
    resetEncoders();
    straighten();
  }

  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public double getRotationPosition() {
    return rotationEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getRotationVelocity() {
    return rotationEncoder.getVelocity();
  }

  public double getAbsolutePosition() {
    return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getAbsolutePositionRad() {
    return getAbsolutePosition() * 2 * Math.PI * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders() {
    //sets drive encoder to set the current position to 0
    //sets the rotation relative encoder sync with the absolute encoder
    driveEncoder.setPosition(0);
    rotationEncoder.setPosition(absoluteEncoder.getAbsolutePosition().getValueAsDouble()*Constants.rotationsToRad);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getRotationPosition()));
  }

  public void straighten() {
    SparkClosedLoopController rotationController = rotationMotor.getClosedLoopController();
    SmartDashboard.putNumber("Setpoint", absoluteEncoderOffset * 2 * Math.PI);
    rotationController.setSetpoint(absoluteEncoderOffset * Constants.rotationsToRad,ControlType.kPosition);
    rotationEncoder.setPosition(0);
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state.optimize(getState().angle);
    driveMotor.set(state.speedMetersPerSecond/Constants.physicalMaxSpeedMetersPerSec);
    SparkClosedLoopController rotationController = rotationMotor.getClosedLoopController();
    rotationController.setSetpoint(state.angle.getRadians(),ControlType.kPosition);
  }

  public void stop() {
    driveMotor.set(0);
    rotationMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    LoggedTunableNumber.ifChanged(
      hashCode(),
      () -> {
        SparkMaxConfig tempConfig = new SparkMaxConfig();
        tempConfig.closedLoop.pid(kPRotation.get(), 0, 0.1);
        rotationMotor.configure(tempConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      },
      kPRotation);
  }
}
