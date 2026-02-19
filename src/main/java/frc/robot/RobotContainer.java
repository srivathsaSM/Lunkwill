package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Constants;
import frc.robot.Commands.GoToAngle;
import frc.robot.Commands.SwerveJoystick;
import frc.robot.Commands.ZeroHeading;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  //note: pushing joystick forward is NEGATIVE Y
  private final Joystick joystick = new Joystick(Constants.kDriverControllerPort);

  public RobotContainer() {
    //in WPILib, positive x = forward and positive y = left
    //to move forward, you need to push the joystick forward, so the xspeed has to be positive when the Y of the joystick is negative (forward)
    //to move left, you need to push the joystick to the left, so the yspeed has to be positive when the x of the joystick is negative (left)
    //twist is just twist
    swerveSubsystem.setDefaultCommand(new SwerveJoystick(
      swerveSubsystem,
      () -> joystick.getRawAxis(Constants.driverYAxis),
      () -> -joystick.getRawAxis(Constants.driverXAxis),
      () -> -joystick.getRawAxis(Constants.driverRotAxis),
      () -> joystick.getRawButton(Constants.driverFieldOrientedButtonIndex))); 
      
    
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(joystick, Constants.zeroHeadingButtonIndex).whileTrue(new ZeroHeading(swerveSubsystem));
    new JoystickButton(joystick, 3).whileTrue(new GoToAngle(swerveSubsystem, Math.PI/2));
  }

  public Command getAutonomousCommand() {
    //Create Trajectory Settings
    // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.physicalMaxSpeedMetersPerSec, Constants.swerveMaxAccel).setKinematics(Constants.driveKinematics);
    
    // //Generate Trajectory 
    // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //   new Pose2d(0,0,new Rotation2d(0),
    //   List.of(new Translation2d(1,0), new Translation2d(1,-1))),
    //   new Pose2d(2,-1, rotation2d.fromDegrees(180)),
    //   trajectoryConfig
    // );

    
    
    return Commands.print("No autonomous command configured");
  }
}
