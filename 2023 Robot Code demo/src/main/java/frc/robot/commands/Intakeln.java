// Gerekli paketleri ve sınıfları içe aktarın
package frc.robot.commands;

import java.util.function.Supplier;
 
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;


// IntakeIn sınıfını oluşturun ve CommandBase sınıfından kalıtım alın
public class Intakeln extends CommandBase {
 
  

    private final Intake intakeSubsystem;
    private final Supplier<Double> controls;

    public Intakeln(Intake intakeSubsystem , Supplier<Double> controls) {
        this.intakeSubsystem = intakeSubsystem;
        this.controls = controls;


        //bu kod satırı commandın hangi subsystema bağlı olduğunu belirtir
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("intake başladı !");
    }

    @Override
    public void execute() {
        double speed=controls.get();
        speed*=0.3;
        intakeSubsystem.setPosition(speed);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("intake bitti !");
        intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
