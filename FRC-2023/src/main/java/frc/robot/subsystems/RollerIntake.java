// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.time.Instant;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RollerIntake extends SubsystemBase {
  
  private static final class Config{
  public static final int kConeIntakeMotorID = 7;
  public static final int kCubeIntakeMotorID = 8;
  public static final double kIntakeSpeed = 0.6;
  }

  /** Creates a new RollerIntake. */
  private CANSparkMax m_coneIntakeMotor = new CANSparkMax(Config.kConeIntakeMotorID, MotorType.kBrushless); 
  //private CANSparkMax m_cubeIntakeMotor = new CANSparkMax(Config.kCubeIntakeMotorID, MotorType.kBrushless);

  public RollerIntake() {
    //m_cubeIntakeMotor.setInverted(true);
    m_coneIntakeMotor.setInverted(true);

    //Sets current limits to 20 (stall) and 30 (free)
    m_coneIntakeMotor.setSmartCurrentLimit(15, 15);
    // m_cubeIntakeMotor.setSmartCurrentLimit(15, 15);
    
    m_coneIntakeMotor.setIdleMode(IdleMode.kBrake);
    // m_cubeIntakeMotor.setIdleMode(IdleMode.kBrake);

    m_coneIntakeMotor.burnFlash();
    // m_cubeIntakeMotor.burnFlash();
  }

  public void setCoastMode() {
    m_coneIntakeMotor.setIdleMode(IdleMode.kCoast);
    // m_cubeIntakeMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setBrakeMode() {
    m_coneIntakeMotor.setIdleMode(IdleMode.kBrake);
    // m_cubeIntakeMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setConeForward(){
    m_coneIntakeMotor.set(Config.kIntakeSpeed);
  }
  
  public InstantCommand intakeCone(){
    return new InstantCommand(this::setConeForward, this);
  }

  public void setCubeForward(){
    // m_cubeIntakeMotor.set(Config.kIntakeSpeed);
  }

  public InstantCommand intakeCube(){
    return new InstantCommand(this::setCubeForward, this);
  }

  public void setConeReverse(){
    m_coneIntakeMotor.set(-Config.kIntakeSpeed);
  }

  public InstantCommand ejectCone(){
    return new InstantCommand(this::setConeReverse, this);
  }

  public void setCubeReverse(){
    // m_cubeIntakeMotor.set(-Config.kIntakeSpeed);
  }

  public InstantCommand ejectCube(){
    return new InstantCommand(this::setCubeReverse, this);
  }

  public void setCubeOff(){
    // m_cubeIntakeMotor.set(0);
  }

  public InstantCommand turnOffCube(){
    return new InstantCommand(this::setCubeOff, this);
  }

  public void setConeOff(){
    m_coneIntakeMotor.set(0);
  }

  public InstantCommand turnOffCone(){
    return new InstantCommand(this::setConeOff, this);
  }

  //Combined intake code
  public void setIntakeIn() {
    m_coneIntakeMotor.set(Config.kIntakeSpeed);
    // m_cubeIntakeMotor.set(Config.kIntakeSpeed);
  }

  public InstantCommand turnOnIntake() {
    return new InstantCommand(this::setIntakeIn, this);
  }

  public void setIntakeOut() {
    m_coneIntakeMotor.set(-Config.kIntakeSpeed);
    // m_cubeIntakeMotor.set(-Config.kIntakeSpeed);
  }

  public InstantCommand turnEjectIntake() {
    return new InstantCommand(this::setIntakeOut, this);
  }
  
  public void setIntakeOff() {
    m_coneIntakeMotor.set(0);
    // m_cubeIntakeMotor.set(0);
  }

  public InstantCommand turnOffIntake() {
    return new InstantCommand(this::setIntakeOff, this);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Cone Motor Temperature", m_coneIntakeMotor.getMotorTemperature());
    // SmartDashboard.putNumber("Cube Motor Temperature", m_cubeIntakeMotor.getMotorTemperature());

    // SmartDashboard.putNumber("Cone Motor Current", m_coneIntakeMotor.getOutputCurrent());
    // SmartDashboard.putNumber("Cone Motor Current", m_coneIntakeMotor.getOutputCurrent());
    // This method will be called once per scheduler run
  }
}
