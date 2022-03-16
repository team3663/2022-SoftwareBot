package frc.robot.utils.Terjectory;

import java.util.ArrayList;
import java.util.List;

import javax.swing.text.html.HTMLDocument.HTMLReader.ParagraphAction;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SwervePath {

    private List<Command> m_startingCommands = new ArrayList<Command>();
    private List<Command> m_endingCommands = new ArrayList<Command>();
    private List<Command> m_initCommands = new ArrayList<Command>();
    private List<Command> m_finishCommands = new ArrayList<Command>();
    private List<Path> m_paths = new ArrayList<Path>();

    private List<Command> m_runCommands = new ArrayList<Command>();

    public DrivetrainSubsystem m_drivetrainSubsystem;

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

    double kpTheta = 0.0004;   // .04
    double kiTheta = 0.0;
    double kdTheta = 0.0;
    ProfiledPIDController thetaController = new ProfiledPIDController(kpTheta, kiTheta, kdTheta, thetaControllerConstraints); 

    


    Command[] commands;
    /**
     * 
     * @param paths List of the paths that you want to make
     * @param StartingCommand Theys commands will run at the *start* of the Path
     * @param LastCommand These Commands will run after the *end* of the Path
    */
    public SwervePath(DrivetrainSubsystem subsystem,List<Path> paths, List<Command> StartingCommand, List<Command> LastCommand){
        m_drivetrainSubsystem = subsystem;
        
        m_paths = paths;
        m_startingCommands = StartingCommand;
        m_endingCommands = LastCommand;
        m_initCommands = List.of();
        m_finishCommands = List.of();
        
        Generate();
    }

    public SwervePath(DrivetrainSubsystem subsystem, List<Path> paths, List<Command> StartingCommand, List<Command> LastCommand, List<Command> InitCommand, List<Command> FinishCommand){
        m_drivetrainSubsystem = subsystem;

        m_paths = paths;
        m_startingCommands = StartingCommand;
        m_endingCommands = LastCommand;
        m_initCommands = InitCommand;
        m_finishCommands = FinishCommand;
    }

    public SwervePath(DrivetrainSubsystem subsystem, List<Path> paths){
        m_drivetrainSubsystem = subsystem;

        m_paths = paths;
        Generate();
    }

    private void Generate(){

        //Adding Init Commands
        if(m_initCommands.size() > 0){

            for(int i =0; i < m_initCommands.size(); i++){
                m_runCommands.add(m_initCommands.get(i));
            }
        }
        
        
        System.out.println(m_paths.size());
        for (int i = 0; i < m_paths.size(); i++) {
            
            //Adding Start Command
            for(Command command : m_startingCommands) {
                m_runCommands.add(command);
            }
            
            //Adding Path Command
            m_runCommands.add(m_paths.get(i).getPathCommand(m_drivetrainSubsystem));
            
            //Adding End Commands
            for (Command command : m_endingCommands) {
                m_runCommands.add(command);
            }
        }
        // Adding Finishing Commands
        if(m_finishCommands.size() > 0){

            for(int i = 0; i < m_finishCommands.size(); i++){
                m_runCommands.add(m_finishCommands.get(i));
            }
        }
        
        //Pushes all the commands into an Array
        commands = new Command[m_runCommands.size()];
        m_runCommands.toArray(commands);
    }


    
    /**
     * 
     * @return Returns all of the paths and starting and ending commands in a SequentialCommandGroup
     */
    public Command Run(){
        return new SequentialCommandGroup(commands);
    }
    
}