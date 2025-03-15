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
  private  PIDController taController;// = new PIDController(0.05, 0, 0);
  private  PIDController tyController;// = new PIDController(0.05, 0, 0);
  private  PIDController txController;//= new PIDController(0.05, 0, 0);

  private final double SET_AREA;
  private final double SET_Y;
  private final double SET_X;

  public AlignWithTag(Swerve drivetrainSubsystem, double area, double y, double x) {
    this.drivetrainSubsystem = drivetrainSubsystem;

    double p = 0.05;
    double i = 0;
    double d = 0;

    double taTolerance= 0.2;
    double tyTolerance= 0.5;
    double txTolerance= 0.4;
//
   taController = new PIDController(p, i, d);
   tyController = new PIDController(p, i, d);
    txController = new PIDController(p, i, d);

    //x was 1
    //y was 10
    //omega was 10
    SET_AREA = area;
    SET_Y = y;
    SET_X = x;

    taController.setTolerance(taTolerance);
   tyController.setTolerance(tyTolerance);
   txController.setTolerance(txTolerance);

   taController.setIntegratorRange(0, 10);
    tyController.setIntegratorRange(-30, 30);
    txController.setIntegratorRange(-30, 30);

    //taController.setTolerance(0.2);
    //tyController.setTolerance(0.5);
    //txController.setTolerance(0.2);
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    //LimelightHelpers.SetFiducialIDFiltersOverride(Constants.Sensor.LIMELIGHT, new int[]{TAG_TO_CHASE});
    LimelightHelpers.SetFiducialDownscalingOverride(Constants.Sensor.LIMELIGHT, 2.0f);
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
      var xSpeed = MathUtil.clamp(taController.calculate(ta, SET_AREA), -0.05, .05);
      if (taController.atSetpoint()) {
        xSpeed = 0;
      }

      var ySpeed =  MathUtil.clamp(tyController.calculate(ty, SET_Y), -0.01, .01);
      if (tyController.atSetpoint()) {
        ySpeed = 0;
      }

      var omegaSpeed = MathUtil.clamp(txController.calculate(tx, SET_X), -0.05, 0.05);
      if (txController.atSetpoint()) {
        omegaSpeed = 0;
      }

      drivetrainSubsystem.drive(new Translation2d(xSpeed,omegaSpeed).times(Constants.Swerve.maxSpeed),0
              ,
              false,
              true);
    }
  }

  @Override
  public boolean isFinished()
  {
    return taController.atSetpoint() && tyController.atSetpoint() && txController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

}
