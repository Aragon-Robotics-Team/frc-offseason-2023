// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class MoveForTime extends CommandBase {
  private static final class Config{
    public static final double kSpeed = 0.4;
  }
  
  private Drivetrain m_drivetrain;
  private Timer m_timer;
  private double m_targetTimeInSecs;
  private double m_speed;
  private boolean m_direction; //FALSE IS BACKWARD, TRUE IS FORWARD

  /** Creates a new MoveForTime. */
  public MoveForTime(Drivetrain drivetrain, double timeInSecs, boolean direction) {
    m_drivetrain = drivetrain;
    m_timer = new Timer();
    m_targetTimeInSecs = timeInSecs;
    m_direction = direction;
    m_speed = Config.kSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  public MoveForTime(Drivetrain drivetrain, double timeInSecs, boolean direction, double speed) {
    m_drivetrain = drivetrain;
    m_timer = new Timer();
    m_targetTimeInSecs = timeInSecs;
    m_direction = direction;
    m_speed = speed;

    m_drivetrain.calibrateGyro();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Auto Forward Timer", m_timer.get());
    if (!m_direction) m_drivetrain.getDrive().arcadeDrive(-m_speed, 0);
    else if (m_direction) m_drivetrain.getDrive().arcadeDrive(m_speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.getDrive().arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_targetTimeInSecs);
  }
}
