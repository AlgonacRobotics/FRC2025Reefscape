// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignToReefLimelightNative extends Command {

  private CommandSwerveDrivetrain drive2;

  private final SwerveRequest.FieldCentric swerveRequest2 = new SwerveRequest.FieldCentric();

  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional()
  {    
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  /** Creates a new AutoAlignToReefLimelightNative. */
  public AutoAlignToReefLimelightNative(CommandSwerveDrivetrain swerve2) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.drive2 = swerve2;
    //Maybe add this.limelight
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve2);
    //Maybe add limelightrequirements

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    final var rot_limelight = limelight_aim_proportional();
        //rot = rot_limelight;

        final var forward_limelight = limelight_range_proportional();
        //xSpeed = forward_limelight;

    drive2.applyRequest(() ->
              swerveRequest2.withVelocityX(forward_limelight) // Drive forward with negative Y (forward), slew code filter.calculate(-joystick.getLeftY())
              .withVelocityY(0) // Drive left with negative X (left)
              .withRotationalRate(rot_limelight) // Drive counterclockwise with negative X (left)
    
        );
      
    //we can test this if the other approach doesn't work
    /* 
    //We apply the speeds to the swerve drive 
    swerveRequest.withVelocityX(xSpeedTY)
    .withVelocityY(0)
    .withRotationalRate(rotTX);

    //use setControl() instead of of applyRequest()
    drive.setControl(swerveRequest);
    */

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
