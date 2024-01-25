// B dosya, asansör alt sistemini tanımlar.

// Geru dosya, ElevatorSubsystem.java dosyasıdır.
// Buekli paketleri ve sınıfları içe aktarın
package frc.robot.subsystems;




import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


// ElevatorSubsystem sınıfını oluşturun ve SubsystemBase sınıfından kalıtım alın
public class Elevator extends SubsystemBase {

  // Asansör motorunu tanımlayın
  private final PWMVictorSPX elevatorMotor = new PWMVictorSPX(Constants.Elevator.kElevatorMotorPort);

  private Encoder elevatorEncoder=new Encoder(6,7,false,EncodingType.k4X);



  // Asansör durumunu takip etmek için bir değişken tanımlayın
  

 
  // Asansörü belirli bir hedefe taşımak için kullanılan metot
  public void moveTo(double sspeed) {
    elevatorMotor.set(sspeed);       
    
 
    // Hareket durumunu true yap
    
  }

  // Asansörü durdurmak için kullanılan metot
  public void stop() {
    // Asansör motorunu durdur
    elevatorMotor.set( 0);

  }

  public double elevatormetre(){
    return (elevatorEncoder.get()*1.0/4096*0.1*Math.PI);
  }
  public boolean elevatorkapali(){
    double error = elevatormetre()-0.05 ;
    return (Math.abs(error) < 0.005);
  }
public boolean elevatororta(){
double error = elevatormetre()-0.5;
return (Math.abs(error)<0.005);
}
public boolean elevatorson(){
  double error = elevatormetre()-1;
  return (Math.abs(error)<0.005);
  }



public void periodic(){

  SmartDashboard.putNumber("Meters: ", elevatormetre()); 
}
}