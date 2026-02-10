package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    
    public static final int kDriverControllerPort = 0;

    //control bindings
    public static final int driverXAxis = 0;
    public static final int driverYAxis = 1;
    public static final int driverRotAxis = 2;
    public static final int driverFieldOrientedButtonIndex = 1;
    public static final int zeroHeadingButtonIndex = 2;

    //swerve module controller and encoder IDs
    public static final int frontRightDriveID = 4;
    public static final int frontRightRotationID = 3;
    public static final int frontRightEncoderID = 40;
    
    public static final int backRightDriveID = 5;
    public static final int backRightRotationID = 6;
    public static final int backRightEncoderID = 30;

    public static final int frontLeftDriveID = 8;
    public static final int frontLeftRotationID = 7;
    public static final int frontLeftEncoderID = 20;

    public static final int backLeftDriveID = 1;
    public static final int backLeftRotationID = 2;
    public static final int backLeftEncoderID = 10;

    //rotation encoder offsets
    //NOTE: CHECK IF THE OFFSETS ARE RADIANS OR ROTATIONS (CONVERSION MESS)
    public static final double frontRightEncoderOffset = 0.713623;
    public static final double backRightEncoderOffset = 0.49707;
    public static final double frontLeftEncoderOffset = 0.795898;
    public static final double backLeftEncoderOffset = -0.60498;

    //drive motor reversed states
    public static final boolean frontRightReversed = false;
    public static final boolean backRightReversed = true;
    public static final boolean frontLeftReversed = true;
    public static final boolean backLeftReversed = false;

    //rotation encoder reversed 
    //NOTE: TRY INVERTING FRONT RIGHT AND BACK LEFT
    public static final boolean frontRightAbsReversed = false;
    public static final boolean backRightAbsReversed = false;
    public static final boolean frontLeftAbsReversed = false;
    public static final boolean backLeftAbsReversed = false;

    //rotation Motor reversed
    public static final boolean frontRightRotReversed = false;
    public static final boolean backRightRotReversed = false;
    public static final boolean frontLeftRotReversed = false;
    public static final boolean backLeftRotReversed = false;


    //conversion factors
    public static final double ticksToRadians = 2*Math.PI/4096;
    public static final double rotationsToRad = 2*Math.PI;

    //swerve PID
    public static final double kPRotation = 0.3;
    public static final double kIRotation = 0;
    public static final double kDRotation = 0;

    //swerve drive hardware specifications
    public static final double wheelDiameterMeters = Units.inchesToMeters(4.0 / 1.04085);
    public static final double driveMotorGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double rotationMotorGearRatio = ((150.0 / 7.0) / 1.0); // 150/7:1
    public static final double driveEncoderRotationToMeters = driveMotorGearRatio * rotationsToRad * wheelDiameterMeters/2;
    public static final double rotationEncoderRotationToRad = rotationMotorGearRatio * rotationsToRad;
    public static final double driveEncoderRPMToMetersPerSec = driveEncoderRotationToMeters/60;
    public static final double rotationEncoderRPMToRadPerSec = rotationEncoderRotationToRad/60;

    public static final double physicalMaxSpeedMetersPerSec = 7.5 / 4.0;
    public static final double maxAngularSpeedRadPerSec = 3.5;

    //deadband
    public static final double deadband = 0.15;

    //limiters
    public static final double swerveMaxAccel = 3;
    public static final double swerveAngularMaxAccel = 3;

    //distance between right and left wheels
    public static final double trackWidth = Units.inchesToMeters(24.5);
    
    //distance between front and back wheels
    public static final double wheelBase = Units.inchesToMeters(18.75);

    //for positioning of swerve modules and calculations
    //order: front left, front right, back left, back right
    public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase/2, -trackWidth/2), //front left
        new Translation2d(wheelBase/2, trackWidth/2),  //front right
        new Translation2d(-wheelBase/2, -trackWidth/2), //back left
        new Translation2d(-wheelBase/2, trackWidth/2));  //back right
}

