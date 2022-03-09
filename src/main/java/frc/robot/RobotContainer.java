// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveFieldCentricCommand;
import frc.robot.commands.DriveHubCentricCommand;
import frc.robot.commands.DriveRobotCentricCommand;
import frc.robot.commands.DriveWithSetRotationCommand;
import frc.robot.commands.VisionDriveToCargo;
import frc.robot.commands.VisionRotateToCargo;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  public final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

  private final XboxController m_controller = new XboxController(0);

  public final SendableChooser<Command> chooser = new SendableChooser<>();

  public Robot m_robot;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(Robot robot) {

    m_robot = robot;
    // set the starting position of the robot on the field
    // TODO: need a chooser object to select starting position and angle
    //DONE in robotInit 
    // drivetrainSubsystem.setGyroscopeHeadingDegrees(0);
    // drivetrainSubsystem.setPose(Constants.ROBOT_1M_LEFT_OF_HUB,
    //     drivetrainSubsystem.getGyroscopeRotation());

    SwerveTrajectoryFollowCommandFactory.addTestTrajectoriesToChooser(chooser, 1.0, 0.75, drivetrainSubsystem, true, m_robot);
    SmartDashboard.putData("Auto mode", chooser);

    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    // drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    //         drivetrainSubsystem,
    //         () -> -modifyAxis(controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> -modifyAxis(controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> -modifyAxis(controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    // ));

    drivetrainSubsystem.setDefaultCommand(new DriveWithSetRotationCommand(
      drivetrainSubsystem, 
      () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
      () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> m_controller.getPOV(), 
      0.0));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    new Button(m_controller::getBackButton)
            // No requirements because we don't need to interrupt anything
            .whenPressed(drivetrainSubsystem::zeroGyroscope);

    new Button(m_controller::getYButton)
            .whenPressed(new DriveWithSetRotationCommand(
              drivetrainSubsystem,
              () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
              () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
              () -> m_controller.getPOV(),
              0));
    new Button(m_controller::getXButton)
            .whenPressed(new DriveHubCentricCommand(drivetrainSubsystem, 
            () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
            () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
            ));

    new Button(m_controller::getBButton)
    .whenPressed(new DriveRobotCentricCommand(drivetrainSubsystem,
    () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * 0.8, 
    () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * 0.8,
    () -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.5));

    new Button(m_controller::getAButton)
        .whenPressed(new DriveFieldCentricCommand(drivetrainSubsystem,
        () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
        () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
      

    new Button(m_controller::getLeftBumper)
      .whileHeld(new VisionRotateToCargo(m_visionSubsystem, drivetrainSubsystem));

    new Button(m_controller::getRightBumper)
      .whileHeld(new VisionDriveToCargo(m_visionSubsystem, drivetrainSubsystem));

    new Button(m_controller::getStartButton)
      .whenPressed(
        new InstantCommand(() -> drivetrainSubsystem.setPose(Constants.StartPoseConstants.BLUE_MID_TOP, Constants.StartPoseConstants.BLUE_MID_TOP.getRotation()), drivetrainSubsystem));
  }

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

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.07);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
