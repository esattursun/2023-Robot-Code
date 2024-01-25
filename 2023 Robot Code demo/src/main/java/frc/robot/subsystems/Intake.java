// Bu dosya, intake.java dosyasıdır.
// Bu dosya, intake alt sistemini tanımlar.

// Gerekli paketleri ve sınıfları içe aktarın
package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


// Intake sınıfını oluşturun ve SubsystemBase sınıfından kalıtım alın
public class Intake extends SubsystemBase {

  // Intake motoru ve açı sensörü için özel değişkenler tanımlayın
  private CANSparkMax intakeMotor = new CANSparkMax(9, MotorType.kBrushless);
  
  private CANSparkMax neoMotor;
 private RelativeEncoder m_angleEncoder;



  
  public Intake() {
    neoMotor= new CANSparkMax(9, MotorType.kBrushless);
    neoMotor.setSmartCurrentLimit(20);
    m_angleEncoder = neoMotor.getEncoder();
  }

  @Override
  public void periodic() {
  }
  public double getEncoderMeters(){
    return m_angleEncoder.getPosition()*-0.1;

  }


  public boolean KAPALI(){
    double error = getEncoderMeters()-0.05 ;
    return (Math.abs(error) < 0.005);
  }

  public boolean ORTA(){
    double error = getEncoderMeters()-4.5 ;
    return (Math.abs(error) < 0.005);
  }

  public boolean SON(){
    double error = getEncoderMeters()-7 ;
    return (Math.abs(error) < 0.005);
  }
  public void setPosition(double speed) {
    intakeMotor.set(speed);
  }
  public void stop(){
    intakeMotor.set(0);
  }
}
