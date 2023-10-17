// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ForwardForTime extends CommandBase {
  /** Creates a new ForwardForTime. */
  public static final class Config {
    private  static double k_wheelSpeed = 0.4;
  }
  private Drivetrain m_drivetrain; 
  private Timer m_timer;
  private double m_time;

  public ForwardForTime(Drivetrain drivetrain, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    m_drivetrain = drivetrain;
    m_timer = new Timer();
    m_time = time;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.getDrive().tankDrive(Config.k_wheelSpeed, Config.k_wheelSpeed);
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.getDrive().tankDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_time);
  }
}
