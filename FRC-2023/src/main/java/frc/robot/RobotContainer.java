
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.subsystems.Music;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// auto sequences
import frc.robot.commands.Autonomous.AutoSequence;
import frc.robot.commands.Autonomous.ScoreLowAuto;
import frc.robot.commands.Autonomous.TwoPiece;
import frc.robot.commands.Autonomous.ReleaseArm;
import frc.robot.commands.Autonomous.ScoreHighAuto;
import frc.robot.commands.Drivetrain.ArcadeDrive;
import frc.robot.commands.Drivetrain.DriveForward;
import frc.robot.commands.Drivetrain.DriveForwardPID;
import frc.robot.commands.Drivetrain.ForwardForTime;
import frc.robot.commands.Drivetrain.SetBrake;
import frc.robot.commands.Drivetrain.SetCoast;
import frc.robot.commands.Drivetrain.DrivetrainIdle;

// import frc.robot.commands.Elevator.EncoderTest;
import frc.robot.commands.Elevator.JoystickElevator;
import frc.robot.commands.Elevator.RecalibrateElevator;
import frc.robot.commands.Elevator.ElevatorExtensionModes.ExtendElevator;
import frc.robot.commands.Elevator.ElevatorExtensionModes.ExtendElevatorSmart;

import frc.robot.commands.Gyro.GyroBalance;
import frc.robot.commands.Autonomous.GyroBalanceNoArm;

import frc.robot.commands.IntakeArm.ArmDown;
import frc.robot.commands.IntakeArm.GoToAngle;
import frc.robot.commands.IntakeArm.GoToAngleSmart;
import frc.robot.commands.IntakeArm.JoystickArm;
import frc.robot.commands.IntakeArm.OpenServo;
import frc.robot.commands.IntakeArm.RecalibrateArm;

import frc.robot.commands.Limelight.AutoTrackPole;
import frc.robot.commands.Limelight.TestServo;
import frc.robot.commands.Limelight.TrackTarget;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.LimelightTestTurret;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final class Config{
    // Joysticks
    public static final int kDriverJoystickID = 0;
    public static final int kSecondJoystickID = 1;

    // Driving 

    /* 
     * there are two driving modes - fast and slow
     * the "axis" is actually a pressure trigger (think button)
     * to toggle between fast driving and slow driving
    */
    public static final int kSlowModeAxis = 3;

    public static final int kSetBrakeID = 5;
    public static final int kSetCoastID = 6;
    // Intake 
    public static final int kIntakeInID = 1;
    public static final int kIntakeOutID = 2;

    // Recalibrate
    public static final int kRecalibrateID = 8; //FOR BOTH ARM AND ELEVATOR
    
    // Scoring
    // (basically arm button ids)
    public static final int kLowScoreID = 5;
    public static final int kHighScoreID = 6;
    public static final int kHumanPlayerID = 3;

    public static final int kRetractID = 4; //FOR BOTH ARM AND ELEVATOR

    // Auto
    public static final double kTimeInSecsFast = 1.1;
    public static final double kTimeInSecsShort = 5.5;
    public static final double kTimeInSecsLong = 7.3;

    /* Driver Controls:
     *  - Left Joystick = Speed
     *  - Right Joystick = Turn
     * 
     * Secondary Controls:
     *  - A = Intake In (release for off)
     *  - B = Intake Out (release for off)
     *  - Y = Recalibrate
     *  - X = Shelf Pickup
     *  - Left Bumper = Low Score
     *  - Right Bumper = High Score
     */
  }


  //Joysticks
  private Joystick m_driveJoystick = new Joystick(Config.kDriverJoystickID); 
  private Joystick m_secondJoystick = new Joystick(Config.kSecondJoystickID);


  // Drivetrain (on Driver Controller)
  private Drivetrain m_drivetrain = new Drivetrain();
  private ArcadeDrive m_arcadeDrive = new ArcadeDrive(m_drivetrain, m_driveJoystick, Config.kSlowModeAxis);

  /* 
   * previously used for testing limelight idle
   * basically to stop the "watchdog not fed" problem
   * is not being used now, but i'll leave it here if we need to test other commands
  */
  // private DrivetrainIdle m_drivetrainIdle = new DrivetrainIdle(m_drivetrain);


  // On Secondary controller
  // Intake
  private RollerIntake m_rollerIntake = new RollerIntake();
  private JoystickButton m_intakeInButton = new JoystickButton(m_secondJoystick, Config.kIntakeInID);
  private JoystickButton m_intakeOutButton = new JoystickButton(m_secondJoystick, Config.kIntakeOutID);


  // Elevator
  private Elevator m_elevator = new Elevator();
  private JoystickElevator m_joystickElevator = new JoystickElevator(m_elevator, m_secondJoystick); //Change to second joystick later

  private RecalibrateElevator m_recalibrateElevator = new RecalibrateElevator(m_elevator);

  // Elevator Extensions: Pass in encoder ticks for specific angle
  private ExtendElevatorSmart m_lowScoreElevator = new ExtendElevatorSmart(m_elevator, -44); //Originally -61
  private ExtendElevatorSmart m_highScoreElevator = new ExtendElevatorSmart(m_elevator, -57);
  private ExtendElevatorSmart m_humanPlayerElevator = new ExtendElevatorSmart(m_elevator, -56);

  // Arm
  private Arm m_arm = new Arm();
  private JoystickArm m_joystickArm = new JoystickArm(m_arm, m_secondJoystick);

  private RecalibrateArm m_recalibrateArm = new RecalibrateArm(m_arm);

  private GoToAngle m_halfAngleArm = new GoToAngle(m_arm, m_arm.getEncoderLimitUp()/2);

  private GoToAngleSmart m_lowScoreArm = new GoToAngleSmart(m_arm, 46.5, m_secondJoystick); //RETEST FOR NEW VALUES
  private GoToAngleSmart m_highScoreArm = new GoToAngleSmart(m_arm, 58, m_secondJoystick);
  private GoToAngleSmart m_humanPlayerArm = new GoToAngleSmart(m_arm, 52, m_secondJoystick);
  private GoToAngleSmart m_retractArm = new GoToAngleSmart(m_arm, 1.5, m_secondJoystick);

  private OpenServo m_openServo = new OpenServo(m_arm);

  //Elevator + Arm Scoring Parallel Groups
  private ParallelCommandGroup m_lowScore = new ParallelCommandGroup(m_lowScoreArm, m_lowScoreElevator);
  private ParallelCommandGroup m_highScore = new ParallelCommandGroup(m_highScoreArm, m_highScoreElevator); 
  private ParallelCommandGroup m_humanPlayer = new ParallelCommandGroup(m_humanPlayerArm, m_humanPlayerElevator);

  private JoystickButton m_lowScoreButton = new JoystickButton(m_secondJoystick, Config.kLowScoreID);
  private JoystickButton m_highScoreButton = new JoystickButton(m_secondJoystick, Config.kHighScoreID);
  private JoystickButton m_humanPlayerButton = new JoystickButton(m_secondJoystick, Config.kHumanPlayerID);
  private JoystickButton m_retractButton = new JoystickButton(m_secondJoystick, Config.kRetractID);

  // recalibration
  private JoystickButton m_recalibrateButton = new JoystickButton(m_secondJoystick, Config.kRecalibrateID);


  // Auto
  private SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  /* 
   * a sendable is able to send data in real time (think updating values on smartDashboard). 
   * a sendable chooser is a sendable with the added functionality of being able to choose
   * between specific options (like inputs into smart dashboard)
   * 
   * the sendable chooser here lets the driver select between different auto sequences 
   */

  // autonomous commands
  private ReleaseArm m_releaseArm = new ReleaseArm(m_arm);
  private AutoSequence m_autoSequenceShort = new AutoSequence(m_drivetrain, m_arm, Config.kTimeInSecsShort);
  private AutoSequence m_autoSequenceLong = new AutoSequence(m_drivetrain, m_arm, Config.kTimeInSecsLong);

  // cube auto actually intakes cones it's just a naming thing
  private ScoreLowAuto m_lowScoreAutoShort = new ScoreLowAuto(m_arm, m_rollerIntake, m_drivetrain, Config.kTimeInSecsShort);
  private ScoreLowAuto m_lowScoreAutoLong = new ScoreLowAuto(m_arm, m_rollerIntake, m_drivetrain, Config.kTimeInSecsLong);
  private ScoreLowAuto m_simpleLowAuto = new ScoreLowAuto(m_arm, m_rollerIntake, m_drivetrain, 0);

  private ScoreHighAuto m_simpleHighAuto = new ScoreHighAuto(m_arm, m_elevator, m_rollerIntake, m_drivetrain, 0);
  private ScoreHighAuto m_highScoreAutoShort = new ScoreHighAuto(m_arm, m_elevator, m_rollerIntake, m_drivetrain, Config.kTimeInSecsFast);

  private GyroBalanceNoArm m_gyroBalanceNoPiece = new GyroBalanceNoArm(m_drivetrain);

  // time is experimental - needs to be tested
  private TwoPiece m_twoPiece = new TwoPiece(m_arm, m_elevator, m_rollerIntake, m_drivetrain, 1.5);

  
  // private Limelight m_limelight = new Limelight();
  // private Turret m_turret = new Turret();
  // private TrackTarget m_track = new TrackTarget(m_limelight, m_turret);
  // private TestServo m_test = new TestServo(m_turret);
  
  // private DriveForward m_driveForward = new DriveForward(m_drivetrain, m_forwardButton);

  private SendableChooser<Boolean> m_pipeChooser = new SendableChooser<>();

  // private GyroBalance m_balance = new GyroBalance(m_drivetrain);

  // for testing
  // private AutoTrackPole m_autoTrack = new AutoTrackPole(m_limelight, m_drivetrain);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // syntax: name, command
    // Taxi/Simple Auto
    //m_autoChooser.setDefaultOption("release_arm_auto", m_releaseArm);
    m_autoChooser.addOption("move_back_auto_short", m_autoSequenceShort); //EVENTUALLY RENAME TO TAXI AUTO
    m_autoChooser.addOption("move_back_auto_long", m_autoSequenceLong); //SEE ABOVE

    // Low Score Autos
    m_autoChooser.addOption("low_score_short", m_lowScoreAutoShort);
    m_autoChooser.addOption("low_score_long", m_lowScoreAutoLong);
    m_autoChooser.addOption("simple_low_auto", m_simpleLowAuto);

    // High Score Autos
    m_autoChooser.addOption("high_auto_short", m_highScoreAutoShort);
    m_autoChooser.addOption("simple_high_auto", m_simpleHighAuto);

    // gyro balance auto (NO PIECE)
    // Robot MUST be facing away from the scoring grid
    m_autoChooser.addOption("gyro_balance_no_piece", m_gyroBalanceNoPiece);
    
    // two piece (needs to be tested)
    m_autoChooser.addOption("two_piece", m_twoPiece);

    SmartDashboard.putData(m_autoChooser);

    // m_pipeChooser.setDefaultOption("Reflective Tape", m_limelight.setPipeline(1));
    // m_pipeChooser.addOption("Cone", m_limelight.setPipeline(2));
    // m_pipeChooser.addOption("Cube", m_limelight.setPipeline(3));
    // SmartDashboard.putData(m_pipeChooser);

    configureButtonBindings();
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Intake on
    m_intakeInButton.onTrue(m_rollerIntake.turnOnIntake());
    m_intakeOutButton.onTrue(m_rollerIntake.turnEjectIntake());

    // Intake off
    m_intakeInButton.onFalse(m_rollerIntake.turnOffIntake());
    m_intakeOutButton.onFalse(m_rollerIntake.turnOffIntake());

    // Recalibrate
    m_recalibrateButton.onTrue(m_recalibrateArm);
    m_recalibrateButton.onTrue(m_recalibrateElevator);

    // Score
    m_lowScoreButton.onTrue(m_lowScore);
    m_highScoreButton.onTrue(m_highScore);
    m_humanPlayerButton.onTrue(m_humanPlayer);

    // Retract Arm/Elevator
    m_retractButton.onTrue(m_retractArm);
    m_retractButton.onTrue(m_recalibrateElevator);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */

public Command getOpenServo(){
  return m_openServo;
}

 public Command getAutonomousCommand() {
    
    // used for testing limelight functionality - testing if it can track reflective tape
    // return m_autoTrack; 

    return m_autoChooser.getSelected(); // choose actual autonomous from smart dashboard
  }

  public Command getTeleopCommand(){
    m_drivetrain.setDefaultCommand(m_arcadeDrive);
    m_elevator.setDefaultCommand(m_joystickElevator);
    m_arm.setDefaultCommand(m_joystickArm);

    // m_limelight.setDefaultCommand(m_track);
    // m_turret.setDefaultCommand(m_test);
    // m_track.schedule();

    // m_encoderTest.schedule(); 
    return null;
  }

  public void setTestCoastMode() {
    m_rollerIntake.setCoastMode();
    m_arm.setCoastMode();
  }

  public void setTestBrakeMode() {
    m_rollerIntake.setBrakeMode();
    m_arm.setBrakeMode();
  }

}
