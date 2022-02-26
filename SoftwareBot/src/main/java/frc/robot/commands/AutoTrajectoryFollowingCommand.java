package frc.robot.commands;

import java.sql.DriverPropertyInfo;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoTrajectoryFollowingCommand extends SwerveControllerCommand{

    private static PIDController xPidController = new PIDController(0.5, 0, 0); 
    private static PIDController yPidController = new PIDController(0, 0, 0); 

    private final static double MAX_PHYSICAL_ANGULAR_VELOCITY = 2 * 2 * Math.PI; // radians per second
    private final static double MAX_ANGULAR_VELOCITY = MAX_PHYSICAL_ANGULAR_VELOCITY / 10;
    private final static double MAX_ANGULAR_ACCELERATION = Math.PI / 4;
    private static TrapezoidProfile.Constraints angleControllerConstraints = new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCELERATION);
    private static ProfiledPIDController anglePidController = new ProfiledPIDController(0.04, 0, 0, angleControllerConstraints);

    public AutoTrajectoryFollowingCommand(DrivetrainSubsystem drivetrainSubsystem, Pose2d start, List<Translation2d> waypoints, Pose2d end) {

        super(drivetrainSubsystem.getTrajectory(start, waypoints, end),
            drivetrainSubsystem::getPose, drivetrainSubsystem.getKinematics(),
            xPidController, yPidController, anglePidController,
            drivetrainSubsystem::setModules,
            drivetrainSubsystem);

        addRequirements(drivetrainSubsystem);

        drivetrainSubsystem.followingTrajectory(true);
    }
}
