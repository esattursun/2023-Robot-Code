package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator ;

public class Elevator_oto_command extends CommandBase {

  private final Elevator elevatorSubsystem;
  private final PIDController pidController;
  private final ElevatorPosition elevatorPosition;

  public static enum ElevatorPosition{
    KAPALI,
    ORTA,
    SON
  }


  public Elevator_oto_command(Elevator elevatorSubsystem, double setPoint, ElevatorPosition elevatorPosition) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.elevatorPosition = elevatorPosition;
    this.pidController = new PIDController(1.5, 0.3,0);
    pidController.setSetpoint(setPoint);
    addRequirements(elevatorSubsystem);
  }

  
  @Override
  public void initialize() {
    pidController.reset();
    System.out.println("ElevatorPID basladı !");
  }

 
  @Override
  public void execute() {
    double speed = pidController.calculate(elevatorSubsystem.elevatormetre());
    elevatorSubsystem.moveTo(speed);
  }

  
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.moveTo(0);
    System.out.println("ElevatorPID bitti !");
  }

  
  @Override
  public boolean isFinished() {
      
    if(elevatorPosition == ElevatorPosition.KAPALI){
        return this.elevatorSubsystem.elevatorkapali();
      
      }else if(elevatorPosition == ElevatorPosition.ORTA){
        return this.elevatorSubsystem.elevatororta();
      
      }else if(elevatorPosition == ElevatorPosition.SON){
        return this.elevatorSubsystem.elevatorson();
      
      }else{
        return false;
      }
  }
}