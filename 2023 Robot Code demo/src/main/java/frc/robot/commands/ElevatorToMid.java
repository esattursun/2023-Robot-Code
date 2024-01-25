// Bu dosya, ElevatorToMid.java dosyasıdır.
// Bu dosya, asansörü orta seviyeye getirmek için verilen bir komutu tanımlar.

// Gerekli paketleri ve sınıfları içe aktarın
package frc.robot.commands;


import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;


// ElevatorToMid sınıfını oluşturun ve CommandBase sınıfından kalıtım alın
public class ElevatorToMid extends CommandBase {
  private final Elevator elevatorSubsystem;
  private final Supplier<Double> controls;

  public ElevatorToMid(Elevator elevatorSubsystem,Supplier<Double> controls) {
      this.elevatorSubsystem = elevatorSubsystem;
      this.controls = controls;

      
      addRequirements(elevatorSubsystem);
  }
  // Elevator alt sistemini kullanmak için bir referans tanımlayın
  

  





  // Komut başlatıldığında çalışacak metodu tanımlayın
  @Override
  public void initialize() {
    System.out.println("asansör başladı");

   
  }

  // Komut devam ederken çalışacak metodu tanımlayın
  @Override
  public void execute() {
    double sspeed= controls.get();
     sspeed*=0.8;
    elevatorSubsystem.moveTo(sspeed);
  }

  

  // Komut bittiğinde çalışacak metodu tanımlayın
  @Override
  public void end(boolean interrupted) {

    System.out.println("asansör durdu");
    // Elevator alt sistemini durdurun
    
    
    
  }
}
