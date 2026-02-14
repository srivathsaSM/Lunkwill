// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveModule;
import frc.robot.Subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToAngle extends Command {
  SwerveSubsystem swerveSubsystem;
  double angleRad;
  public GoToAngle(SwerveSubsystem swerveSubsystem, double angleRad) {
    this.swerveSubsystem = swerveSubsystem;
    this.angleRad = angleRad;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Rotation2d angle = new Rotation2d(angleRad);
    // SwerveModuleState[] states = new SwerveModuleState[4];
    // for (int i = 0; i < 4; i++) {
    //   states[i] = new SwerveModuleState(0,angle);
    // }
    // swerveSubsystem.setModuleStates(states);
    SwerveModule[] modules = swerveSubsystem.getModules();
    for (SwerveModule module : modules) {
      module.straighten();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
