
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;

// import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Drivetrain extends SubsystemBase {
  
  /** Creates a new Drivetrain. */
  private static final class Config {
    public static final int kLeftPrimaryID = 1;
    public static final int kRightPrimaryID = 3;
    public static final int kLeftSecondaryID = 2;
    public static final int kRightSecondaryID = 4;
  }

  
  private WPI_TalonFX m_leftPrimary = new WPI_TalonFX(Config.kLeftPrimaryID);
  private WPI_TalonFX m_rightPrimary = new WPI_TalonFX(Config.kRightPrimaryID);
  private WPI_TalonFX m_leftSecondary = new WPI_TalonFX(Config.kLeftSecondaryID);
  private WPI_TalonFX m_rightSecondary = new WPI_TalonFX(Config.kRightSecondaryID);
  private DifferentialDrive m_drive = new DifferentialDrive(m_leftPrimary, m_rightPrimary);
  private double m_initAngle = 0.0;

  // private ADXRS450_Gyro m_gyro  = new ADXRS450_Gyro();
  private AHRS m_gyro  = new AHRS(SPI.Port.kMXP);


  public Drivetrain() {
    m_leftSecondary.follow(m_leftPrimary);
    m_rightSecondary.follow(m_rightPrimary);
    
    m_rightPrimary.setInverted(false);
    m_rightSecondary.setInverted(false);
    m_leftPrimary.setInverted(true);
    m_leftSecondary.setInverted(true);
  }


  public DifferentialDrive getDrive(){
    return m_drive;
  }
 
  public double getTicks(){
    return m_leftPrimary.getSelectedSensorPosition();
  }

  public void resetTicks(){
    m_leftPrimary.setSelectedSensorPosition(0.0);
  }

  public void calibrateGyro(){
    m_gyro.calibrate();
  }

  // yaw will change if the robot spins around in a circle
  public double getTotalAngle(){
    return m_gyro.getAngle();
  }

  public double getAngle(){
    // return m_gyro.getAngle() - 360*(int)(m_gyro.getAngle()/360);
    return getRoll();
  }


  // if you tilt the robot sideways (not the way it goes up on charge station) pitch will move accordingly
  public double getPitch(){
    return m_gyro.getPitch(); 
  }

  // roll will change if the robot tilts upward (whilst moving foward, e.g. going up the charge station)
  public double getRoll(){
    return m_gyro.getRoll();
  }

  public double getRegRoll(){
    return m_gyro.getRoll() - 360*(int)(m_gyro.getAngle()/360);
  }
 
  public void setBrake(){
    m_leftPrimary.setNeutralMode(NeutralMode.Brake);
    m_rightPrimary.setNeutralMode(NeutralMode.Brake);
    m_leftSecondary.setNeutralMode(NeutralMode.Brake);
    m_rightSecondary.setNeutralMode(NeutralMode.Brake);
  }

  public void setCoast(){
    m_leftPrimary.setNeutralMode(NeutralMode.Coast);
    m_rightPrimary.setNeutralMode(NeutralMode.Coast);
    m_leftSecondary.setNeutralMode(NeutralMode.Coast);
    m_rightSecondary.setNeutralMode(NeutralMode.Coast);
  }

  private void registerInitAngle() {
   m_initAngle = getAngle(); 
  }

  public InstantCommand getRegisterInitAngle() {
    return new InstantCommand(this::registerInitAngle);
  }

  public double getInitAngle() {
    return m_initAngle;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Roll angle", getRoll());
    SmartDashboard.putNumber("Yaw angle", getAngle());
    SmartDashboard.putNumber("Pitch Angle", getPitch());
    // This method will be called once per scheduler run
  }
}
