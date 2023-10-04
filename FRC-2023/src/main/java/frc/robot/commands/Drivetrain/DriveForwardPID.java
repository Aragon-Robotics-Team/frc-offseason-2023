// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveForwardPID extends CommandBase {

  private static final class Config{
    public static final double kP = 0.05;
    public static final double kI = 0.05;
    public static final double kD = 0.05;

    public static final double kTicksToFeet = (6*Math.PI)/2048;
  }
  private Drivetrain m_drivetrain;
  private PIDController m_controller = new PIDController(Config.kP, Config.kI, Config.kD);
  
  // where is this
  private double m_setpoint = 10*12;
  
  /** Creates a new DriveForwardPID. */
  public DriveForwardPID(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   m_drivetrain.resetTicks();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = m_controller.calculate(m_drivetrain.getTicks(), m_setpoint);
    m_drivetrain.getDrive().tankDrive(speed, speed, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   m_drivetrain.getDrive().tankDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drivetrain.getTicks()*Config.kTicksToFeet - m_setpoint <= 0.1 && m_drivetrain.getTicks()*Config.kTicksToFeet - m_setpoint >= -0.1;
  }
}
