// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatic;

public class PneumaticCommand extends CommandBase {

  private final Pneumatic pneumaticSubsystem;
  private final boolean SolenoidOn;
  private final boolean SolenoidOff;
   
  public PneumaticCommand(Pneumatic pneumaticSubsystem, boolean SolenoidOn, boolean SolenoidOff) {
    this.pneumaticSubsystem = pneumaticSubsystem;
    this.SolenoidOn = SolenoidOn;
    this.SolenoidOff = SolenoidOff;
    addRequirements(pneumaticSubsystem);
  }

  @Override
  public void initialize() {}

 
  @Override
  public void execute() {
    pneumaticSubsystem.SolenoidForward(SolenoidOn);
    pneumaticSubsystem.SolenoidReverse(SolenoidOff);
  }

  
  @Override
  public void end(boolean interrupted) {
  }

 
  @Override
  public boolean isFinished() {
    if(SolenoidOff){
      return true;
    }else if(SolenoidOn){
      return true;
    }else{
      return false;
    }
  }
}