package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */

  private final SparkMax driveMotor;
  private final SparkMax rotationMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder rotationEncoder;

  private final CANcoder absoluteEncoder;

  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffset;

  public SwerveModule(int driveMotorID, int rotationMotorID, int absoluteEncoderID, boolean absoluteEncoderReversed, double absoluteEncoderOffset, boolean driveInverted, boolean rotationInverted) {
    //motors
    this.driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
    this.rotationMotor = new SparkMax(rotationMotorID, MotorType.kBrushless);

    //Encoders
    this.driveEncoder = driveMotor.getEncoder();
    this.rotationEncoder = rotationMotor.getEncoder();
    this.absoluteEncoder = new CANcoder(absoluteEncoderID);

    //drive config
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig.inverted(driveInverted);
    driveConfig.encoder.positionConversionFactor((Math.PI * Constants.wheelDiameterMeters)/Constants.driveMotorGearRatio);
    driveConfig.encoder.velocityConversionFactor((Math.PI * Constants.wheelDiameterMeters)/(60 * Constants.driveMotorGearRatio));
    this.driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //rotation config
    SparkMaxConfig rotationConfig = new SparkMaxConfig();
    rotationConfig.inverted(rotationInverted);
    rotationConfig.closedLoop.pid(Constants.kPRotation,Constants.kIRotation, Constants.kDRotation);
    rotationConfig.closedLoop.positionWrappingEnabled(true);
    rotationConfig.closedLoop.positionWrappingMaxInput(Math.PI);
    rotationConfig.closedLoop.positionWrappingMinInput(-Math.PI);
    rotationConfig.encoder.positionConversionFactor(2*Math.PI/Constants.rotationMotorGearRatio);
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
    return absoluteEncoder.getAbsolutePosition().getValueAsDouble() - absoluteEncoderOffset;
  }

  public double getAbsolutePositionRad() {
    return getAbsolutePosition() * 2 * Math.PI * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders() {
    driveEncoder.setPosition(0);
    rotationEncoder.setPosition(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getRotationPosition()));
  }

  public void straighten() {
    SparkClosedLoopController rotationController = rotationMotor.getClosedLoopController();
    rotationController.setSetpoint(absoluteEncoderOffset*Constants.rotationsToRad,ControlType.kPosition);
    rotationEncoder.setPosition(0);
  }

  public void setDesiredState(SwerveModuleState state) {
    state.optimize(getState().angle);
    driveMotor.set(state.speedMetersPerSecond/Constants.physicalMaxSpeedMetersPerSec);
    SparkClosedLoopController rotationController = rotationMotor.getClosedLoopController();
    rotationController.setSetpoint(state.angle.getRadians(),ControlType.kPosition);
    //if rotation on swerve modules resets after every time we stop moving joysticks, refer to 9:40
  }

  public void stop() {
    driveMotor.set(0);
    rotationMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
