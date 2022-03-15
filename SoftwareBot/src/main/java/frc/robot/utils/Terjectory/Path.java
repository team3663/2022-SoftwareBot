package frc.robot.utils.Terjectory;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Path{
	
	private TrajectoryConfig m_config;
	private List<Translation2d> m_points = new ArrayList<Translation2d>();
	private List<Command> m_parallelCommands = new ArrayList<Command>();
	private Pose2d m_endPose = new Pose2d();

	private boolean m_isParallelCommand;


    double kpX = 0.00000000000001;  //.01 for both X and Y
    double kpY = 0.00000000000001;
    double kiX = 0.0;
    double kiY = 0.0;
    double kdX = 0.0;
    double kdY = 0.0;
    PIDController xController = new PIDController(kpX, kiX, kdX); 
    PIDController yController = new PIDController(kpY, kiY, kdY);
	
    double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    double kMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
    double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;

    TrapezoidProfile.Constraints thetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond,
        kMaxAngularAccelerationRadiansPerSecondSquared);

	double kpTheta = 0.0001;   // .04
    double kiTheta = 0.0;
    double kdTheta = 0.0;
    ProfiledPIDController thetaController = new ProfiledPIDController(kpTheta, kiTheta, kdTheta, thetaControllerConstraints); 


	public Path(Pose2d StartPose, List<Translation2d> Pointe, Pose2d EndPose, double Acceleration, double MaxSpeed){
		m_isParallelCommand = false;
		m_config = new TrajectoryConfig(MaxSpeed, Acceleration);
		m_points = Pointe;
		m_endPose = EndPose;
	}
	
	public Path(Pose2d StartPose,List<Command> ParallelCommands, List<Translation2d> Pointe, Pose2d EndPose, double Acceleration, double MaxSpeed){
		m_isParallelCommand = true;
		m_config = new TrajectoryConfig(MaxSpeed, Acceleration);
		m_points = Pointe;
		m_endPose = EndPose;
	}

	public Trajectory getTrajectory(){
		return TrajectoryGenerator.generateTrajectory(new Pose2d(0,0, new Rotation2d(0)),m_points,m_endPose, m_config);
	}

	public boolean isParallelCommand(){
		return m_isParallelCommand;
	}

	public Command getPathCommand(DrivetrainSubsystem m_drivetrainSubsystem) {
		System.out.println("-------Create Command--------");
		return new SwerveControllerCommand(
			getTrajectory(),
			m_drivetrainSubsystem::getPose,
			m_drivetrainSubsystem.getKinematics(),
			xController,
			yController,
			thetaController,
			m_drivetrainSubsystem::setModuleStates,
			m_drivetrainSubsystem);
	}

	public Command getCommand(DrivetrainSubsystem m_drivetrainSubsystem,List<Command> commands){
		commands.add(new SwerveControllerCommand(
			getTrajectory(),
			m_drivetrainSubsystem::getPose,
			m_drivetrainSubsystem.getKinematics(),
			xController,
			yController,
			thetaController,
			m_drivetrainSubsystem::setModuleStates,
			m_drivetrainSubsystem));
		Command command_array[] = new Command[commands.size()];
		commands.toArray(command_array);
		return new ParallelCommandGroup(command_array);	
	}
}
