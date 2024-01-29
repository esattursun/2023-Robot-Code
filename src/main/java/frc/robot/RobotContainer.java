package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.elevator_command;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.Elevator_oto_command;
import frc.robot.commands.Intake_oto_command;
import frc.robot.commands.Intakeln;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.commands.PneumaticCommand;
import frc.robot.commands.Elevator_oto_command.ElevatorPosition;
import frc.robot.commands.Intake_oto_command.IntakePosition;
import frc.robot.poseestimation.PoseEstimation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class RobotContainer {
    // Alt sistemler
    private final Joystick m_gamepad = new Joystick(1);
    public static final Drivetrain drivetrain = new Drivetrain();
    private final static Elevator elevator = new Elevator();
    private final Intake intake = new Intake();
    public final static PoseEstimation poseEstimation = new PoseEstimation();
    private final PneumaticSubsystem pneumatic = new  PneumaticSubsystem();


  public static Field2d field = new Field2d();
      public static final ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
   // private final DriveSubsystem m_robotDrive =new DriveSubsystem();
    //private final Joystick m_gamepad2 = new Joystick(2);
   
     


    public RobotContainer() {
        configureButtonBindings();
         
        elevator.setDefaultCommand(new elevator_command(elevator, 
        ()-> m_gamepad.getRawAxis(1)));
        intake.setDefaultCommand(new Intakeln(intake, 
        ()-> -m_gamepad.getRawAxis(5)));

        drivetrain.setDefaultCommand(new DriveWithJoysticks(drivetrain, poseEstimation, m_gamepad));
    }
/* 
        m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_gamepad2.getRawAxis(1)*0.5, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_gamepad2.getRawAxis(0)*0.5, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_gamepad2.getRawAxis(2)*0.5, OIConstants.kDriveDeadband),
                    true, true),
                m_robotDrive));
      }
     */
        
    
    
    private void configureButtonBindings(){
        new JoystickButton(m_gamepad,5).whileTrue(new PneumaticCommand(pneumatic,true,false));
        new JoystickButton(m_gamepad,6).whileTrue(new PneumaticCommand(pneumatic,false,true));

        new JoystickButton(m_gamepad, 1).toggleOnTrue(new SequentialCommandGroup(
            new Elevator_oto_command(elevator, -0.05 , ElevatorPosition.KAPALI)));

            new JoystickButton(m_gamepad, 3).toggleOnTrue(new SequentialCommandGroup(
                new Elevator_oto_command(elevator, -0.5 , ElevatorPosition.ORTA)));

                 
                new JoystickButton(m_gamepad, 4).toggleOnTrue(new SequentialCommandGroup(
                    new Elevator_oto_command(elevator, -1 , ElevatorPosition.SON)));

                     new JoystickButton(m_gamepad, 2).toggleOnTrue(new SequentialCommandGroup(
                new Intake_oto_command(intake, -7 , IntakePosition.SON)));
          
    }

   
    public Command getAutonomousCommand() {
       return null;
          }
    




    public static Elevator getLiftSubsystem() {
        return elevator;
    }
}