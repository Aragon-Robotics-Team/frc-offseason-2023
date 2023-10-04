// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.Autonomous;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drivetrain.TurnToAngle;
import frc.robot.commands.Elevator.RecalibrateElevator;
import frc.robot.commands.Elevator.ElevatorExtensionModes.ExtendElevatorSmart;
import frc.robot.commands.IntakeArm.GoToAngleSmart;
import frc.robot.commands.IntakeArm.RecalibrateArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.RollerIntake;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html


public class TwoPiece extends SequentialCommandGroup {
  /** Creates a new TwoPiece. */


  private final double moveTime;


  // this time things needs to be tested at comp
  public TwoPiece(Arm arm, Elevator elevator, RollerIntake rollerIntake, Drivetrain drivetrain, double time) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());




    // move arm up
    // roller out
    // recalibrate arm
    // move backward to piece
    // turn 180
    // with arm down - roller in
    // turn 180
    // move for some distance
    // arm up
    // roller out
    moveTime = 2.0;


    addCommands(


      // score piece and move off
      // copied from ScoreHighAuto
      new ReleaseArm(arm),
      new RecalibrateArm(arm),
      new ParallelRaceGroup(new GoToAngleSmart(arm, 63), new ExtendElevatorSmart(elevator, -59), new ParallelCommandGroup(new WaitCommand(2.85), new RunIntakeIn(rollerIntake))),
      new ParallelCommandGroup(new MoveForTime(drivetrain, 0.6, true), new RunIntakeIn(rollerIntake)), //MAKE SAME TIME AS BACK
      new WaitCommand(0.1),
      new RunIntakeOut(rollerIntake),
      new MoveForTime(drivetrain, 0.6, false), //MAKE SAME TIME AS FORWARD


      // causes some DT differential drive error with motor output and not putting data enough
      new ParallelCommandGroup(new RecalibrateArm(arm), new RecalibrateElevator(elevator)),


      new MoveForTime(drivetrain, time, false), // ticked speed constant from 0.6 => 0.7
      new TurnToAngle(drivetrain, 180),
      // i have no clue how long this is -- needs to be tested!!
      new MoveForTime(drivetrain, moveTime, true),

      
      // piece intake
      new RunIntakeIn(rollerIntake),


      new TurnToAngle(drivetrain, 180),
      new MoveForTime(drivetrain, 0.6 + moveTime, true, 0.7),


      // copied from low
      new ReleaseArm(arm),
      new RecalibrateArm(arm),
      new ParallelRaceGroup(new GoToAngleSmart(arm, 30.0), new WaitCommand(3.0)), //ISSUE
      new RunIntakeOut(rollerIntake),
      new MoveForTime(drivetrain, time, false),
      new RecalibrateArm(arm),


      new MoveForTime(drivetrain, time, false, 0.7)


      // robot should now be off of tarmac with two pieces placed
    );
  }
}
