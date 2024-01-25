// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class Intake_oto_command extends CommandBase {

  private final Intake intakeSubsystem;
  private final PIDController pidController;  
  private final IntakePosition intakePosition;

  public static enum IntakePosition{
    KAPALI,
    ORTA,
    SON
  }

  /** Creates a new AutoIntake. */
  public Intake_oto_command(Intake intakeSubsystem, double setPoint, IntakePosition intakePosition) {
    this.intakeSubsystem = intakeSubsystem;
    this.intakePosition = intakePosition;
    this.pidController = new PIDController(1.2, .3, .1);
    pidController.setSetpoint(setPoint);
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
    System.out.println("IntakePIDCmd Basladi!");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidController.calculate(intakeSubsystem.getEncoderMeters());
    intakeSubsystem.setPosition(-speed * .5);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setPosition(0);
    System.out.println("IntakePIDCmd kapali!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(intakePosition == IntakePosition.KAPALI){
      return this.intakeSubsystem.KAPALI();

    }else if(intakePosition == IntakePosition.ORTA){
      return this.intakeSubsystem.ORTA();

    }else if(intakePosition == IntakePosition.SON){
      return this.intakeSubsystem.SON();
    }else{
      return false;
    }
  }
}