package frc.robot.utils.Terjectory;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.utils.Terjectory.*;


public class AutoDrive {
	DrivetrainSubsystem m_subsystem;
	double defaultExsel = 1;
	double defaultspeed = 1;

	public List<Path> paths = new ArrayList<Path>();
	public SwervePath path;
	public List<Command> StartingCommands = new ArrayList<Command>();
	public List<Command> EndingCommands = new ArrayList<Command>();
	

	public void addPath(Path path){
		paths.add(path);
	}

	public void addPath(Pose2d startPose, Pose2d endPose, List<Translation2d> inbetween){
		paths.add(new Path(startPose,inbetween,endPose,defaultExsel, defaultspeed));
	}

	public AutoDrive(DrivetrainSubsystem subsystem, List<Command> StartingCommands, List<Command> EndingCommands) {
		m_subsystem = subsystem;
		this.StartingCommands = StartingCommands;
		this.EndingCommands = EndingCommands;

	}

	public SwervePath getPath(){
		path = new SwervePath(m_subsystem, paths, StartingCommands, EndingCommands);
		return path;
	}

	
}