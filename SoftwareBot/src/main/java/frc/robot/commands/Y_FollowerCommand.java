// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Y_FollowerCommand extends CommandBase {
  private DrivetrainSubsystem drivetrain;
  private PIDController xController = new PIDController(5, 0, 0);
  private PIDController yController = new PIDController(5, 0, 0);
  private TrapezoidProfile.Constraints constraints = new Constraints(1, 1);
  private ProfiledPIDController tController = new ProfiledPIDController(5, 0, 0, constraints);
  private HolonomicDriveController controller = new HolonomicDriveController(xController, yController, tController);

  private Trajectory.State targetState;
  private Rotation2d targetRotation;
  
  public Y_FollowerCommand(DrivetrainSubsystem drivetrain, Trajectory trajectory, double targetAngle) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    tController.enableContinuousInput(0, Math.PI * 2);

    controller.setEnabled(true);
    controller.setTolerance(new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(2)));

    targetState = trajectory.sample(trajectory.getTotalTimeSeconds());
    targetRotation = Rotation2d.fromDegrees(targetAngle);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();
    ChassisSpeeds chassis = controller.calculate(currentPose, targetState, targetRotation);
    drivetrain.drive(chassis);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return controller.atReference();
  }
}
