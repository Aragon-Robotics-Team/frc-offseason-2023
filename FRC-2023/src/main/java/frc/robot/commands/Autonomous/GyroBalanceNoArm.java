// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.Gyro.GyroBalance;


import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html


public class GyroBalanceNoArm extends SequentialCommandGroup {
  /** Creates a new GyroAutoBalance. */
  public GyroBalanceNoArm(Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(
      // add arm thingy

      /*
       * NOTE:
       * For this current auto sequence, I assume the bot is facing FORWARD - arm is extending TOWARDS the charge station and AWAY from the scoring grid.
       */

      // new ParallelCommandGroup(new GyroBalance(drivetrain), new SequentialCommandGroup(new WaitCommand(0.5), new MoveForTime(drivetrain, 2, true)))
      new SequentialCommandGroup(new WaitCommand(0.5), drivetrain.getRegisterInitAngle(), new MoveForTime(drivetrain, 4, true), new GyroBalance(drivetrain))
    );
  }
}
