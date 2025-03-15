package frc.robot.commands.movement;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

public class AlignWithTag extends Command {

  //private static final int TAG_TO_CHASE = 2;

  private final Swerve drivetrainSubsystem;
  private final PIDController taController;
  private final PIDController tyController;
  private final PIDController txController;

  private final double SET_AREA;
  private final double SET_Y;
  private final double SET_X;

  public AlignWithTag(Swerve drivetrainSubsystem, double area, double y, double x) {
    this.drivetrainSubsystem = drivetrainSubsystem;


    taController = new PIDController(0.5, 0, 0.1);
    tyController = new PIDController(0.5, 0, 0.1);
    txController = new PIDController(0.5, 0, 0.1);

    taController.setIntegratorRange(-0.05, 0.05);
    tyController.setIntegratorRange(-0.05, 0.05);
    txController.setIntegratorRange(-0.05, 0.05);

    //x was 1
    //y was 10
    //omega was 10
    SET_AREA = area;
    SET_Y = y;
    SET_X = x;

    taController.setTolerance(0.2);
    tyController.setTolerance(0.5);
    txController.setTolerance(0.5);

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    //LimelightHelpers.SetFiducialIDFiltersOverride(Constants.Sensor.LIMELIGHT, new int[]{TAG_TO_CHASE});
    LimelightHelpers.SetFiducialDownscalingOverride(Constants.Sensor.LIMELIGHT, 2.0f);
    //xController.setSetpoint(5);
    //yController.setSetpoint(0);
  }

  @Override
  public void execute() {
    //Check if Limelight is seeing a target


    if (!LimelightHelpers.getTV(Constants.Sensor.LIMELIGHT)) {
      // No target has been visible
      drivetrainSubsystem.stop();
    }
    else
    {
      double tx = LimelightHelpers.getTX(Constants.Sensor.LIMELIGHT);  // Horizontal offset from crosshair to target in degrees
      double ty = LimelightHelpers.getTY(Constants.Sensor.LIMELIGHT);  // Vertical offset from crosshair to target in degrees
      double ta = LimelightHelpers.getTA(Constants.Sensor.LIMELIGHT);

      // Drive to the target
      var xSpeed = MathUtil.clamp(taController.calculate(ta, SET_AREA), -0.1, .1);
      if (taController.atSetpoint()) {
        xSpeed = 0;
      }

      var turnSpeed =  MathUtil.clamp(tyController.calculate(ty, SET_Y), -0.01, .01);
      if (tyController.atSetpoint()) {
        turnSpeed = 0;
      }

      var ySpeed = MathUtil.clamp(txController.calculate(tx, SET_X), -0.1, 0.1);
      if (txController.atSetpoint()) {
        ySpeed = 0;
      }

      drivetrainSubsystem.drive(new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed),0
              ,
              false,
              false);
    }
  }

  @Override
  public boolean isFinished()
  {
    return taController.atSetpoint()&& txController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

}
