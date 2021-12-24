// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * This class implements field centric swerve drive, with fixed rotational control.
 * The robot defaults to zero degree rotation. Pressing the Xbox POV buttons change
 * the target angle.
 * 
 * Inspired by Team 1684's comprehensive whitepaper on Swerve.
 * https://www.first1684.com/uploads/2/0/1/6/20161347/chimiswerve_whitepaper__2_.pdf 
 * and Team 2910's 2021 competition robot code
 * https://github.com/FRCTeam2910/2021CompetitionRobot
 */

public class DriveWithSetRotationCommand extends CommandBase {
  private final DrivetrainSubsystem m_drivetrainSubsystem;

  // input suppliers from joysticks
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final DoubleSupplier m_rotationPOVSupplier;

  // robot rotation in radians to hold while driving
  private double m_setRotationRadians;

  // PID controller to maintain fixed rotation.
  // TODO: maybe add TrapezoidProfile like in WPILib example:
  //    https://github.com/wpilibsuite/allwpilib/blob/2ad2d2ca9628ab4130135949c7cea3f71fd5d5b6/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervecontrollercommand/subsystems/SwerveModule.java#L27-L34
  private PIDController rotationController = new PIDController(3.0, 0.0, 0.02);

  /**
   * 
   * DriveWithSetRotationCommand - field centric swerve drive, while holding a given robot rotation
   * 
   * @param drivetrainSubsystem
   * @param translationXSupplier
   * @param translationYSupplier
   * @param rotationRadians
   */
  public DriveWithSetRotationCommand(DrivetrainSubsystem drivetrainSubsystem,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationPOVSupplier,
      double rotationRadians) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_translationXSupplier = translationXSupplier;
    m_translationYSupplier = translationYSupplier;
    m_rotationPOVSupplier = rotationPOVSupplier;
    m_setRotationRadians = rotationRadians;

    // TODO: this or .enableContinousInput(-Math.PI, Math.PI); ? maybe needs to match swerve modules
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    SmartDashboard.putNumber("TargetAngle", Math.toDegrees(m_setRotationRadians));
    SmartDashboard.putNumber("RobotAngle", m_drivetrainSubsystem.getGyroscopeRotation().getDegrees());
    SmartDashboard.putNumber("RotationOutput", 0.0);

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController.reset();
    rotationController.setSetpoint(m_setRotationRadians);

    SmartDashboard.putNumber("DriveWithRationAngle", m_setRotationRadians);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double pov = m_rotationPOVSupplier.getAsDouble();
    if (pov >= 0.0) {
      // pov of -1 indicates no POV button pressed

      if (pov > 180) {
        pov = 360 - pov;
      }
      else {
        pov = - pov;
      }
      // convert POV angle from degrees to Radians
      m_setRotationRadians = Math.toRadians(pov);

      System.out.println("POV=" + pov);
      SmartDashboard.putNumber("TargetAngle", Math.toDegrees(m_setRotationRadians));

      // reset the PID controller
      rotationController.reset();
    }

    double rotationOutput = rotationController.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getRadians(), m_setRotationRadians);

    SmartDashboard.putNumber("TargetAngle", Math.toDegrees(m_setRotationRadians));
    SmartDashboard.putNumber("RobotAngle", m_drivetrainSubsystem.getGyroscopeRotation().getDegrees());
    SmartDashboard.putNumber("RotationOutput", rotationOutput);

    m_drivetrainSubsystem.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
              m_translationXSupplier.getAsDouble(),
              m_translationYSupplier.getAsDouble(),
              rotationOutput,
              m_drivetrainSubsystem.getGyroscopeRotation()
      )
    );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
