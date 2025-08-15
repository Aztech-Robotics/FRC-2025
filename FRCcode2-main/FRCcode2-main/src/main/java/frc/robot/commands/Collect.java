// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Collect extends Command {
  /** Creates a new throw. */
  private final Intake mIntake;
  private final ElevatorSub mElevatorSub;
  double mtime;
  boolean Flag = false;
  public Collect(Intake mIntake, ElevatorSub mElevatorSub) {
    this.mElevatorSub = mElevatorSub;
    this.mIntake = mIntake;
addRequirements(mIntake);
addRequirements(mElevatorSub);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    mtime = Timer.getFPGATimestamp();
    if (mtime >= 0.1) {
mElevatorSub.setPosElevator(0);
      mIntake.ConstanVel(1);
      mIntake.SetPosM(4.15);     
    }
    if (mtime>=3.5){
      Flag = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Flag;
  }
}
