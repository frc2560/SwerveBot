package frc.robot.subsystems.Controllers;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorControllerSubsystem extends SubsystemBase {
    public Joystick operatorController = new Joystick(Constants.ControllerConstants.OPERATOR_PORT);
    public Trigger rightWhiteButton;
    public Trigger rightBlackButton;
    public Trigger rightRedButton;
    public Trigger rightBlueButton;
    public Trigger rightYellowButton;
    public Trigger rightGreenButton;
    public Trigger leftWhiteButton;
    public Trigger leftBlackButton;
    public Trigger leftRedButton;
    public Trigger leftBlueButton;
    public Trigger leftYellowButton;
    public Trigger leftGreenButton;

    public OperatorControllerSubsystem() {
        rightWhiteButton = new JoystickButton(operatorController, Constants.ControllerConstants.ButtonConstants.rightWhiteButton);
        rightBlackButton = new JoystickButton(operatorController, Constants.ControllerConstants.ButtonConstants.rightBlackButton);
        rightRedButton = new JoystickButton(operatorController, Constants.ControllerConstants.ButtonConstants.rightRedButton);
        rightBlueButton = new JoystickButton(operatorController, Constants.ControllerConstants.ButtonConstants.rightBlueButton);
        rightYellowButton = new JoystickButton(operatorController, Constants.ControllerConstants.ButtonConstants.rightYellowButton);
        rightGreenButton = new JoystickButton(operatorController, Constants.ControllerConstants.ButtonConstants.rightGreenButton);

        leftWhiteButton = new JoystickButton(operatorController, Constants.ControllerConstants.ButtonConstants.leftWhiteButton);
        leftBlackButton = new JoystickButton(operatorController, Constants.ControllerConstants.ButtonConstants.leftBlackButton);
        leftRedButton = new JoystickButton(operatorController, Constants.ControllerConstants.ButtonConstants.leftRedButton);
        leftBlueButton = new JoystickButton(operatorController, Constants.ControllerConstants.ButtonConstants.leftBlueButton);
        leftYellowButton = new JoystickButton(operatorController, Constants.ControllerConstants.ButtonConstants.leftYellowButton);
        leftGreenButton = new JoystickButton(operatorController, Constants.ControllerConstants.ButtonConstants.leftGreenButton);
    }
}

