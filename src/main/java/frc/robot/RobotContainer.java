package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.coral.*;
import frc.robot.commands.algae.*;
import frc.robot.commands.elevator.*;
import frc.robot.commands.movement.AlignWithTag;
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


    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();
    public final CoralSubsystem coralSubsystem = new CoralSubsystem();
    public final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public final OperatorControllerSubsystem operatorControllerSubsystem = new OperatorControllerSubsystem();
    public final DriverControllerSubsystem driverControllerSubsystem = new DriverControllerSubsystem();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);

        /*
        NamedCommands.registerCommand("AlignToTag", new AlignWithTag(s_Swerve));

        NamedCommands.registerCommand("ScoreOnLevel4", new Level4ScoreCommand(s_Swerve));
        NamedCommands.registerCommand("GrabAlgaeL1", new GrabAlgaeL1Command(s_Swerve));
        NamedCommands.registerCommand("IntakeCoral", new CoralIntakeCommand(coralSubsystem));
        NamedCommands.registerCommand("ScoreInProcessor", new ProcessorScoreCommand(s_Swerve));
        NamedCommands.registerCommand("GrabAlgaeL2", new GrabAlgaeL2Command(s_Swerve));
        NamedCommands.registerCommand("KnockAlgaeOffL1", new KnockAlgaeOffL1Command(s_Swerve));
        NamedCommands.registerCommand("KnockAlgaeOffL2", new KnockAlgaeOffL2Command(s_Swerve));
        */

        driverControllerSubsystem.button2.whileTrue(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driverControllerSubsystem.GetXRawAxis(),
                        () -> driverControllerSubsystem.GetYRawAxis(),
                        () -> (driverControllerSubsystem.GetZRawAxis() * 0.25),
                        () -> driverControllerSubsystem.triggerButton.getAsBoolean()
                )
        );

            s_Swerve.setDefaultCommand(
                    new TeleopSwerve(
                            s_Swerve,
                            () -> -driverControllerSubsystem.GetXRawAxis(),
                            () -> driverControllerSubsystem.GetYRawAxis(),
                            () -> 0,
                            () -> driverControllerSubsystem.triggerButton.getAsBoolean()
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
        driverControllerSubsystem.button3.whileTrue(new AlignWithTag(s_Swerve, Constants.AlignToTag.RightReef_AREA_STAGE, Constants.AlignToTag.RightReef_Y_STAGE, Constants.AlignToTag.RightReef_OMEGA_STAGE));
        driverControllerSubsystem.button4.whileTrue(new AlignWithTag(s_Swerve, Constants.AlignToTag.RightReef_AREA_REEF, Constants.AlignToTag.RightReef_Y_REEF, Constants.AlignToTag.RightReef_OMEGA_REEF));
        driverControllerSubsystem.button12.whileTrue(Commands.run(s_Swerve::resetBot));

        operatorControllerSubsystem.leftYellowButton.whileTrue(new AlgaeIntakeCommand(algaeSubsystem));
        operatorControllerSubsystem.leftGreenButton.whileTrue(new AlgaeOutTakeCommand(algaeSubsystem));
        operatorControllerSubsystem.rightYellowButton.whileTrue(new RaiseAlgaeArmCommand(algaeSubsystem));
        operatorControllerSubsystem.rightGreenButton.whileTrue(new LowerAlgaeArmCommand(algaeSubsystem));

        operatorControllerSubsystem.leftBlueButton.whileTrue(new CoralIntakeCommand(coralSubsystem));
        operatorControllerSubsystem.leftRedButton.whileTrue(new CoralOutTakeCommand(coralSubsystem));
        operatorControllerSubsystem.rightRedButton.whileTrue(new SetCoralArmL4Command(coralSubsystem));
        operatorControllerSubsystem.rightBlueButton.whileTrue(new SetCoralArmL1L2L3Command(coralSubsystem));

        operatorControllerSubsystem.leftBlackButton.whileTrue(new SetCoralArmFeederCommand(coralSubsystem));
        operatorControllerSubsystem.leftWhiteButton.whileTrue(new RaiseCoralArmCommand(coralSubsystem));
      //  operatorControllerSubsystem.rightBlackButton.whileTrue(new GoToL3Command(elevatorSubsystem));
       // operatorControllerSubsystem.rightWhiteButton.whileTrue(new GoToL4Command(elevatorSubsystem));

        driverControllerSubsystem.button7.whileTrue(new GoToBottomCommand(elevatorSubsystem));
        driverControllerSubsystem.button8.whileTrue(new GoToL1Command(elevatorSubsystem));
        driverControllerSubsystem.button9.whileTrue(new GoToL3Command(elevatorSubsystem));
        driverControllerSubsystem.button10.whileTrue(new GoToL4Command(elevatorSubsystem));


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
