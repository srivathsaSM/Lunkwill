package frc.robot.Subsystems;

import org.littletonrobotics.junction.Logger;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(
          Constants.frontLeftDriveID, 
          Constants.frontLeftRotationID, 
          Constants.frontLeftEncoderID,
          Constants.frontLeftAbsReversed,
          Constants.frontLeftEncoderOffset,
          Constants.frontLeftReversed,
          Constants.frontLeftRotReversed);

  private final SwerveModule frontRight = new SwerveModule(
          Constants.frontRightDriveID, 
          Constants.frontRightRotationID, 
          Constants.frontRightEncoderID,
          Constants.frontRightAbsReversed,
          Constants.frontRightEncoderOffset,
          Constants.frontRightReversed,
          Constants.frontRightRotReversed);

  private final SwerveModule backLeft = new SwerveModule(
          Constants.backLeftDriveID, 
          Constants.backLeftRotationID, 
          Constants.backLeftEncoderID,
          Constants.backLeftAbsReversed,
          Constants.backLeftEncoderOffset,
          Constants.backLeftReversed,
          Constants.backLeftRotReversed);
  
  private final SwerveModule backRight = new SwerveModule(
          Constants.backRightDriveID, 
          Constants.backRightRotationID, 
          Constants.backRightEncoderID,
          Constants.backRightAbsReversed,
          Constants.backRightEncoderOffset,
          Constants.backRightReversed,
          Constants.backRightRotReversed);

  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  
  public SwerveSubsystem() {
      new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {
      }
    }).start();
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Robot Heading", getHeading());

    SmartDashboard.putNumber("Rot FR", frontRight.getAbsolutePositionRad());
    SmartDashboard.putNumber("Rot FL", frontLeft.getAbsolutePositionRad());
    SmartDashboard.putNumber("Rot BR", backRight.getAbsolutePositionRad());
    SmartDashboard.putNumber("Rot BL", backLeft.getAbsolutePositionRad());

    SmartDashboard.putNumber("State Radians FR: ", frontRight.getState().angle.getRadians());
    SmartDashboard.putNumber("Actual Radians FR: ", frontRight.getRotationPosition());

    SmartDashboard.putNumber("State Radians FL: ", frontLeft.getState().angle.getRadians());
    SmartDashboard.putNumber("Actual Radians FL: ", frontLeft.getRotationPosition());

    SmartDashboard.putNumber("State Radians BR: ", backRight.getState().angle.getRadians());
    SmartDashboard.putNumber("Actual Radians BR: ", backRight.getRotationPosition());

    SmartDashboard.putNumber("State Radians BL: ", backLeft.getState().angle.getRadians());
    SmartDashboard.putNumber("Actual Radians BL: ", backLeft.getRotationPosition());
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public SwerveModule[] getModules() {
    SwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};
    return modules;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.physicalMaxSpeedMetersPerSec);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }
}
