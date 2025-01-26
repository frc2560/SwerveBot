package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis = 2;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, 11);
//    private final JoystickButton setWheelsToZero = new JoystickButton(driver, 12);
    private final JoystickButton zeroPose = new JoystickButton(driver, 10);
    private final JoystickButton robotCentric = new JoystickButton(driver, 1);
    private final JoystickButton enableZ = new JoystickButton(driver, 2);

    private final JoystickButton resetPose = new JoystickButton(driver, 12);

    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        enableZ.whileTrue(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> (-driver.getRawAxis(rotationAxis) * 0.5),
                        () -> robotCentric.getAsBoolean()
                )
        );

            s_Swerve.setDefaultCommand(
                    new TeleopSwerve(
                            s_Swerve,
                            () -> -driver.getRawAxis(translationAxis),
                            () -> -driver.getRawAxis(strafeAxis),
                            () -> 0,
                            () -> robotCentric.getAsBoolean()
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
        zeroGyro.onTrue(new InstantCommand(s_Swerve::zeroHeading));
//        setWheelsToZero.onTrue(new InstantCommand(s_Swerve::alignStraight));
        zeroPose.onTrue(new InstantCommand(s_Swerve::zeroHeading));
        resetPose.onTrue((new InstantCommand((s_Swerve::resetBot))));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous

        return new PathPlannerAuto("Test Auto");
        //return new ExampleCommand(s_Swerve);
    }
}
