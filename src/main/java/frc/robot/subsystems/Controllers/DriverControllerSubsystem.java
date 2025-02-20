package frc.robot.subsystems.Controllers;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;

import javax.naming.ldap.Control;

public class DriverControllerSubsystem extends SubsystemBase {
    //Driver Joystick
    public Joystick driverController = new Joystick(ControllerConstants.DRIVER_PORT);
    public Trigger triggerButton;
    public Trigger button2;
    public Trigger button3;
    public Trigger button4;
    public Trigger button5;
    public Trigger button6;
    public Trigger button7;
    public Trigger button8;
    public Trigger button9;
    public Trigger button10;
    public Trigger button11;
    public Trigger button12;
    public Trigger zeroButton;
    public Trigger ninetyButton;
    public Trigger oneEightyButton;
    public Trigger twoSeventy;


    public DriverControllerSubsystem() {

        //driver Joystick

        triggerButton = new JoystickButton(driverController, ControllerConstants.ButtonConstants.Trigger);
        button2 = new JoystickButton(driverController, ControllerConstants.ButtonConstants.Button_2);
        button3 = new JoystickButton(driverController, ControllerConstants.ButtonConstants.Button_3);
        button4 = new JoystickButton(driverController, ControllerConstants.ButtonConstants.Button_4);
        button5 = new JoystickButton(driverController, ControllerConstants.ButtonConstants.Button_5);
        button6 = new JoystickButton(driverController, ControllerConstants.ButtonConstants.Button_6);
        button7 = new JoystickButton(driverController, ControllerConstants.ButtonConstants.Button_7);
        button8 = new JoystickButton(driverController, ControllerConstants.ButtonConstants.Button_8);
        button9 = new JoystickButton(driverController, ControllerConstants.ButtonConstants.Button_9);
        button10 = new JoystickButton(driverController, ControllerConstants.ButtonConstants.Button_10);
        button11 = new JoystickButton(driverController, ControllerConstants.ButtonConstants.Button_11);
        button12 = new JoystickButton(driverController, ControllerConstants.ButtonConstants.Button_12);

        //POV on Joystick
        zeroButton = new POVButton(driverController, 0);
        ninetyButton = new POVButton(driverController, 90);
        oneEightyButton = new POVButton(driverController, 180);
        twoSeventy = new POVButton(driverController, 270);


    }
    public double GetXRawAxis() {

        return driverController.getRawAxis(ControllerConstants.AxesConstants.translationAxis);
    }

    public double GetYRawAxis() {
        return
                -driverController.getRawAxis(ControllerConstants.AxesConstants.strafeAxis);
    }

    public double GetZRawAxis() {
        return -driverController.getRawAxis(ControllerConstants.AxesConstants.rotationAxis);
    }

    public double GetSliderRawAxis() {
        return ((driverController.getRawAxis(ControllerConstants.AxesConstants.SLIDER)*-1)+1)/2;
    }

    // Driving


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
