// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    SlewRateLimiter filter = new SlewRateLimiter(0.75);

    SlewRateLimiter filter2 = new SlewRateLimiter(0.5);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController m_operatorStick = new CommandXboxController(1);

    //subsystems 
    public static final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

    public static final WristSubsystem m_wristSubsystem = new WristSubsystem();

    public static final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

    public final CommandSwerveDrivetrain drivetrain;

    // Configure Autonomous chooser 
     //SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        drivetrain = TunerConstants.createDrivetrain();
        configureBindings();
        //autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`

        NamedCommands.registerCommand("elevatorCoralL2", new InstantCommand(
        () -> m_elevatorSubsystem.moveToPosition(Constants.Elevator.elevatorCoralL2)));//good

    NamedCommands.registerCommand("wristCoralL2", new InstantCommand(
        () -> m_wristSubsystem.moveToPosition(Constants.Wrist.wristCoralL2)));//good

    NamedCommands.registerCommand("wristTravel", new InstantCommand(
        () -> m_wristSubsystem.moveToPosition(Constants.Wrist.wristTravel)));//good

    NamedCommands.registerCommand("intakeOpen", new InstantCommand(
        () -> m_intakeSubsystem.moveToPosition(Constants.Intake.intakeOpen)));//good


        autoChooser = AutoBuilder.buildAutoChooser("Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);
        //AutoCommand = AutoBuilder.buildAuto("Auto");
        //configureAutoCommands();
       
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(filter.calculate(-joystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward), slew code filter.calculate(-joystick.getLeftY())
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //joystick.b().whileTrue(drivetrain.applyRequest(() ->
          //  point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        //));

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(0.5).withVelocityY(0))
    );
    joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(-0.5).withVelocityY(0))
    );


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Make drive slow button
        //.x().onTrue(new InstantCommand(
          //  () -> drivetrain.enableSlowSpeed()));

        // Make drive normal button
        //joystick.b().onTrue(new InstantCommand(
          //  () -> drivetrain.enableNormalSpeed()));


        //configure operatorStick button bindings

        //SCORE L3 move wrist to coral LM then move elevator to coral medium
        m_operatorStick.b().onTrue(Commands.sequence(
            new InstantCommand(
            () -> m_wristSubsystem.moveToPosition(Constants.Wrist.wristTravel)),
            new WaitCommand(1),
            new InstantCommand(
            () -> m_elevatorSubsystem.moveToPosition(Constants.Elevator.elevatorCoralL3)),
            new WaitCommand(2),
            new InstantCommand(
            () -> m_wristSubsystem.moveToPosition(Constants.Wrist.wristCoralL3))
            ));

        //SCORE L1 move wrist to coral LM then move elevator to home position
        m_operatorStick.a().onTrue(Commands.sequence(
            new InstantCommand(
            () -> m_wristSubsystem.moveToPosition(Constants.Wrist.wristTravel)),
            new WaitCommand(1),
            new InstantCommand(
            () -> m_elevatorSubsystem.moveToPosition(Constants.Elevator.elevatorCoralL2)),
            new WaitCommand(2),
            new InstantCommand(
            () -> m_wristSubsystem.moveToPosition(Constants.Wrist.wristCoralL2))
            ));

        //SCORE L2 move wrist to coral LM then move elevator to coral low
        m_operatorStick.x().onTrue(Commands.sequence(
            new InstantCommand(
            () -> m_wristSubsystem.moveToPosition(Constants.Wrist.wristTravel)),
            new WaitCommand(1),
            new InstantCommand(
            () -> m_elevatorSubsystem.moveToPosition(Constants.Elevator.elevatorCoralL2)),
            new WaitCommand(2),
            new InstantCommand(
            () -> m_wristSubsystem.moveToPosition(Constants.Wrist.wristCoralL2))
            ));

        //SCORE L4 move wrist to coral H then move elevator to coral high
        /*m_operatorStick.y().onTrue(Commands.sequence(
            new InstantCommand(
            () -> m_wristSubsystem.moveToPosition(Constants.Wrist.wristCoralH))//,
            //new InstantCommand(
            //() -> m_elevatorSubsystem.moveToPosition(Constants.Elevator.elevatorCoralHigh))
             ));*/

        //COLLECTION POSITION move elevator home then move Wrist collect position 
        m_operatorStick.povDown().onTrue(Commands.sequence(
            new InstantCommand(
            () -> m_wristSubsystem.moveToPosition(Constants.Wrist.wristTravel)),
            new WaitCommand(1),
            new InstantCommand(
            () -> m_elevatorSubsystem.moveToPosition(Constants.Elevator.elevatorHomePosition)),
            new WaitCommand(2),
            new InstantCommand(
            () -> m_wristSubsystem.moveToPosition(Constants.Wrist.wristCollect)),
            new InstantCommand(
            () -> m_intakeSubsystem.moveToPosition(Constants.Intake.intakeClose))
            ));


            //move Wrist coral Low Medium position
        //m_operatorStick.povRight().onTrue(new InstantCommand(
            //() -> m_wristSubsystem.moveToPosition(Constants.Wrist.wristCoralLM)));

        //move Wrist coral high position
       // m_operatorStick.povUp().onTrue(new InstantCommand(
           // () -> m_wristSubsystem.moveToPosition(Constants.Wrist.wristCoralH)));

        //Open Intake
        m_operatorStick.rightBumper().onTrue(new InstantCommand(
            () -> m_intakeSubsystem.moveToPosition(Constants.Intake.intakeOpen)));

        //Close Intake
        m_operatorStick.leftBumper().onTrue(new InstantCommand(
            () -> m_intakeSubsystem.moveToPosition(Constants.Intake.intakeClose)));    
    }

    private void configureAutoCommands(){
    //build auto path commands
    NamedCommands.registerCommand("ElevatorCoralLow", new RunCommand(
        () -> m_elevatorSubsystem.moveToPosition(Constants.Elevator.elevatorCoralL2)));

    NamedCommands.registerCommand("WristCoralLM", new RunCommand(
        () -> m_wristSubsystem.moveToPosition(Constants.Wrist.wristCoralL2)));

    NamedCommands.registerCommand("WristCollectL1", new RunCommand(
        () -> m_wristSubsystem.moveToPosition(Constants.Wrist.wristCollect)));

    NamedCommands.registerCommand("IntakeOpen", new RunCommand(
        () -> m_intakeSubsystem.moveToPosition(Constants.Intake.intakeOpen)));

    NamedCommands.registerCommand("ElevatorCoralHigh", new RunCommand(
        () -> m_elevatorSubsystem.moveToPosition(Constants.Elevator.elevatorCoralHigh)));

    NamedCommands.registerCommand("WristCoralH", new RunCommand(
        () -> m_wristSubsystem.moveToPosition(Constants.Wrist.wristCoralL3)));

        /************ Alliance Barge Path ************
     *
     * starts on line on alliance side and scores coral on level 2 side
     *
     */
    //Command allianceBargeAuto = new PathPlannerAuto("AllianceBargeAuto");
    //m_chooser.addOption("Alliance Barge Side Auto", allianceBargeAuto);

    /************ Center Path ************
     *
     * starts on line on alliance side and scores coral on level 1 center
     *
     */
    //Command centerAuto = new PathPlannerAuto("CenterAuto");
    //m_chooser.setDefaultOption("Center Auto", centerAuto);

     /************ Opposing Alliance Path ************
     *
     * starts on line on Opposing alliance side and scores coral on level 2 side
     *
     */
  //  PathPlannerAuto opposingAllianceAuto = new PathPlannerAuto("OpposingAllianceAuto");
  
 //SmartDashboard.putData("Opposing Alliance Barge Side Auto", new PathPlannerAuto("OpposingAllianceAuto"));
  //autoChooser = AutoBuilder.buildAutoChooser();

  //autoChooser = AutoBuilder.buildAutoChooser("OpposingAllianceAuto");
    //m_chooser.addOption("Opposing Alliance Barge Side Auto", opposingAllianceAuto);


    //SmartDashboard.putData("Auto choices:", m_chooser);
    
    }
    //Command AutoCommand; 
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        //return AutoCommand;
        //return Commands.print("No autonomous command configured");
    }
}
