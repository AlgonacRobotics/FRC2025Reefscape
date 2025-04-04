// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignToReef extends Command {

  private CommandSwerveDrivetrain drive; 
  // Maybe add limelight helpers or limelight subsystem

  // define PID controlers.
  private final PIDController rotatePID = new PIDController(0.1, 0, 0);
  private final PIDController turnPID = new PIDController(0.1, 0, 0);
  private final PIDController forwardPID = new PIDController(0.1, 0, 0);

  private final SwerveRequest.FieldCentric swerveRequest = new SwerveRequest.FieldCentric();
  
  /** Creates a new AutoAlignToReef. */
  public AutoAlignToReef(CommandSwerveDrivetrain swerve) {
    this.drive = swerve;
    //Maybe add this.limelight
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
    //Maybe add limelightrequirements

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotatePID.reset();
    turnPID.reset();
    forwardPID.reset();

    rotatePID.setTolerance(Math.toRadians(1));
    turnPID.setTolerance(1); 
    forwardPID.setTolerance(1);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Validate valid target and potentally reef id tags 

    double tx = LimelightHelpers.getTX("limelight");
    double ty = LimelightHelpers.getTY("limelight");

    var rotTX = rotatePID.calculate(tx,0);
    var xSpeedTY = rotatePID.calculate(ty,0);

    drive.applyRequest(() ->
              swerveRequest.withVelocityX(-xSpeedTY) // Drive forward with negative Y (forward), slew code filter.calculate(-joystick.getLeftY())
              .withVelocityY(0) // Drive left with negative X (left)
              .withRotationalRate(-rotTX) // Drive counterclockwise with negative X (left)
    
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
