// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.ModuleLayer.Controller;
import java.util.function.Supplier;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

//TODO: make a better name for this command
public class DriveJoystickRotationCommand extends CommandBase {

  DrivetrainSubsystem m_drivetrain;

  Supplier<Double> m_xSupplier;
  Supplier<Double> m_ySupplier;
  Supplier<Double> m_leftXSupplier;
  Supplier<Double> m_leftYSupplier;

  double stickX;
  double stickY;
  double angle;

  // PID controller to keep a constant rotation
  private ProfiledPIDController rotationController = new ProfiledPIDController(1.5, 0, 0,
    new TrapezoidProfile.Constraints(DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
      DrivetrainSubsystem.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED * 0.9));

  /** Creates a new DriveJoystickRotationCommand. */
  public DriveJoystickRotationCommand(DrivetrainSubsystem drivetrain, Supplier<Double> xSupplier, Supplier<Double> ySupplier,
      Supplier<Double> leftXSupplier, Supplier<Double> leftYSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_xSupplier = xSupplier;
    m_ySupplier = ySupplier;
    m_leftXSupplier = leftXSupplier;
    m_leftYSupplier = leftYSupplier;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // initialize the x and y position of the joystick
    stickX = deadband( m_xSupplier.get(), 0.5 );
    stickY = deadband( m_ySupplier.get(), 0.5 );

    // find the angle using these x and y positions
    angle = Math.acos( stickX / stickY );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(m_leftXSupplier.get(), m_leftYSupplier.get(),
      rotationController.calculate(m_drivetrain.getGyroscopeRotation().getRadians(), angle),
      m_drivetrain.getGyroscopeRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(m_leftXSupplier.get(), m_leftYSupplier.get(),
    0, m_drivetrain.getGyroscopeRotation()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // the command finishes once the robot is at the desired rotation
    return ( (m_drivetrain.getGyroscopeRotation().getRadians() == angle) );
  }

  // the deadband prevents drivers from accidentally rotating the robot
  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }
}
