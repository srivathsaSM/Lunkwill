package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Constants;
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
      () -> -joystick.getRawAxis(Constants.driverYAxis),
      () -> -joystick.getRawAxis(Constants.driverXAxis),
      () -> joystick.getRawAxis(Constants.driverRotAxis),
      () -> !joystick.getRawButton(Constants.driverFieldOrientedButtonIndex))); //this should be NOT so that the field oriented is on by defualt
      //removing the NOT makes robot oriented drive on by default (good for testing)
    
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(joystick, Constants.zeroHeadingButtonIndex).onTrue(new ZeroHeading(swerveSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
