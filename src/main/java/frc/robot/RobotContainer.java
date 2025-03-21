package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.algae.LowerAlgaeArmCommand;
import frc.robot.commands.algae.RaiseAlgaeArmCommand;
import frc.robot.commands.climber.LowerClimberArmCommand;
import frc.robot.commands.climber.RaiseClimberArmCommand;
import frc.robot.commands.coral.*;
import frc.robot.commands.elevator.*;
import frc.robot.commands.grabAlgae.GoAlgaeL1Command;
import frc.robot.commands.grabAlgae.GoAlgaeL2Command;
import frc.robot.commands.grabAlgae.KnockAlgaeCommand;
import frc.robot.commands.scoreCoral.*;
import frc.robot.commands.scoreCoral.Left.LeftScoreCoralL2Command;
import frc.robot.commands.scoreCoral.Left.LeftScoreCoralL3Command;
import frc.robot.commands.scoreCoral.Left.LeftScoreCoralL4Command;
import frc.robot.commands.scoreCoral.Right.RightScoreCoralL2Command;
import frc.robot.commands.scoreCoral.Right.RightScoreCoralL3Command;
import frc.robot.commands.scoreCoral.Right.RightScoreCoralL4Command;
import frc.robot.commands.movement.TeleopSwerve;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Controllers.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private final SendableChooser<Command> autoChooser;
    public boolean isAutonomous = false;


    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();
    public final CoralSubsystem coralSubsystem = new CoralSubsystem();
    public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    public final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public final OperatorControllerSubsystem operatorControllerSubsystem = new OperatorControllerSubsystem();
    public final DriverControllerSubsystem driverControllerSubsystem = new DriverControllerSubsystem();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Build an auto chooser. This will use Commands.none() as the default option.


        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");


        NamedCommands.registerCommand("ScoreOnLevel4Right", new RightScoreCoralL4Command(coralSubsystem, elevatorSubsystem, s_Swerve, isAutonomous));
        NamedCommands.registerCommand("ScoreOnLevel3Right", new RightScoreCoralL3Command(coralSubsystem,elevatorSubsystem, s_Swerve ,isAutonomous));
        NamedCommands.registerCommand("ScoreOnLevel4Left", new LeftScoreCoralL4Command(coralSubsystem, elevatorSubsystem, s_Swerve, isAutonomous));
        NamedCommands.registerCommand("ScoreOnLevel3Left", new LeftScoreCoralL3Command(coralSubsystem,elevatorSubsystem, s_Swerve, isAutonomous));
        NamedCommands.registerCommand("CoralFromFeeder", new GetCoralFeederCommandGroup(coralSubsystem, elevatorSubsystem, s_Swerve));
        NamedCommands.registerCommand("ResetPose", (Commands.run(s_Swerve::resetBot)));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        driverControllerSubsystem.button2.whileTrue(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driverControllerSubsystem.GetXRawAxis(),
                        () -> driverControllerSubsystem.GetYRawAxis(),
                        () -> (driverControllerSubsystem.GetZRawAxis() * 0.25),
                        () -> false
                )
        );

            s_Swerve.setDefaultCommand(
                    new TeleopSwerve(
                            s_Swerve,
                            () -> -driverControllerSubsystem.GetXRawAxis(),
                            () -> driverControllerSubsystem.GetYRawAxis(),
                            () -> 0,
                            () -> false
                    )
            );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {


        /* Driver Buttons */
        driverControllerSubsystem.triggerButton.whileTrue(new TriggerScoreCommand(coralSubsystem, elevatorSubsystem));
        driverControllerSubsystem.button3.whileTrue(new LeftScoreCoralL3Command(coralSubsystem, elevatorSubsystem, s_Swerve, isAutonomous));
        driverControllerSubsystem.button4.whileTrue(new RightScoreCoralL3Command(coralSubsystem, elevatorSubsystem, s_Swerve, isAutonomous));
        driverControllerSubsystem.button5.whileTrue(new LeftL4SetUpCommandGroup(elevatorSubsystem, s_Swerve, isAutonomous));
        driverControllerSubsystem.button6.whileTrue(new RightL4SetUpCommandGroup(elevatorSubsystem, s_Swerve, isAutonomous));

        driverControllerSubsystem.button7.whileTrue(new GoToBottomCommand(elevatorSubsystem));
        driverControllerSubsystem.button8.whileTrue(new GoToL1Command(elevatorSubsystem));
        driverControllerSubsystem.button9.whileTrue(new GoToL4Command(elevatorSubsystem));
        driverControllerSubsystem.button10.whileTrue(Commands.run(s_Swerve::resetBot));
        driverControllerSubsystem.button11.whileTrue(new LeftScoreCoralL2Command(coralSubsystem, elevatorSubsystem, s_Swerve, isAutonomous));

        driverControllerSubsystem.button12.whileTrue(new RightScoreCoralL2Command(coralSubsystem, elevatorSubsystem, s_Swerve, isAutonomous));



        operatorControllerSubsystem.leftGreenButton.whileTrue(new GetCoralFeederCommandGroup(coralSubsystem, elevatorSubsystem, s_Swerve));
        operatorControllerSubsystem.leftYellowButton.whileTrue(new DownFromFeederCommandGroup(coralSubsystem, elevatorSubsystem));
        operatorControllerSubsystem.rightYellowButton.whileTrue(new RaiseCoralArmCommand(coralSubsystem));
        operatorControllerSubsystem.rightGreenButton.whileTrue(new SetCoralArmFeederCommand(coralSubsystem));

        operatorControllerSubsystem.leftRedButton.whileTrue(new CoralIntakeCommand(coralSubsystem));
        operatorControllerSubsystem.leftBlueButton.whileTrue(new CoralOutTakeCommand(coralSubsystem));
        operatorControllerSubsystem.rightBlueButton.whileTrue(new SetCoralArmL1L2L3Command(coralSubsystem));
        //operatorControllerSubsystem.rightBlueButton.whileTrue(new SetCoralArmL1L2L3Command(coralSubsystem));
        operatorControllerSubsystem.rightRedButton.whileTrue(new SetCoralArmL4Command(coralSubsystem));

        operatorControllerSubsystem.leftWhiteButton.whileTrue(new RaiseAlgaeArmCommand(algaeSubsystem));
        operatorControllerSubsystem.leftBlackButton.whileTrue(new LowerAlgaeArmCommand(algaeSubsystem));
        //operatorControllerSubsystem.rightBlackButton.whileTrue(new GetCoralFeederCommandGroup(coralSubsystem, elevatorSubsystem, s_Swerve));
        //operatorControllerSubsystem.rightWhiteButton.whileTrue(new DownFromFeederCommandGroup(coralSubsystem, elevatorSubsystem));





    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
        //return new ExampleCommand(s_Swerve);
    }
}
