// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // added definisions

  private final XboxController controller = new XboxController(0);

  PIDController pid = new PIDController(.00001, .000001, 0);

  // motor names
  private final WPI_TalonFX motor_FRang = new WPI_TalonFX(3);

  private final WPI_TalonFX motor_FRmag = new WPI_TalonFX(2);

  private final WPI_TalonFX motor_FLang = new WPI_TalonFX(8);
  private final WPI_TalonFX motor_FLmag = new WPI_TalonFX(4);

  private final WPI_TalonFX motor_RRang = new WPI_TalonFX(1);
  private final WPI_TalonFX motor_RRmag = new WPI_TalonFX(6);

  private final WPI_TalonFX motor_RLang = new WPI_TalonFX(5);
  private final WPI_TalonFX motor_RLmag = new WPI_TalonFX(7);

  CANCoder FR_coder = new CANCoder(9);
  CANCoder FL_coder = new CANCoder(11);
  CANCoder RR_coder = new CANCoder(10);
  CANCoder RL_coder = new CANCoder(12);

  //

  // motor controll vectors

  private double FRX;
  private double FRY;
  private double FRtangent = 315;

  private double FLX;
  private double FLY;
  private double FLtangent = 45;

  private double RRX;
  private double RRY;
  private double RRtangent = 135;

  private double RLX;
  private double RLY;
  private double RLtangent = 225;

  // resultant vectors
  private double FRAng;
  private double FRMag;

  private double FLAng;
  private double FLMag;

  private double RRAng;
  private double RRMag;

  private double RLAng;
  private double RLMag;
  //

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    pid.enableContinuousInput(0, 360);
    FR_coder.setPositionToAbsolute();
    FR_coder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // drive(controller.getLeftX(), controller.getLeftY(), controller.getRightX());
    // Coeficent = -.36
    pid.setTolerance(50);
    double X;
    double Y = controller.getLeftY();
    pid.setSetpoint(controller.getLeftX() * 180);
    if (pid.onTarget()) {
      X = 0;
    } else {
      X = MathUtil.clamp(pid.calculate(FR_coder.getPosition()), -.05, .05);
    }

    motor_FRmag.set((-.36 * X));
    motor_FRang.set(X);

    // motor_FLmag.set(controller.getLeftY());
    // motor_RRmag.set(controller.getLeftY());
    // motor_RLmag.set(controller.getLeftY());

    // controller.getLeftX()), -1, 1));
    // motor_FLang.set(pid.calculate(FL_coder.getPosition(),controller.getLeftX()));
    // motor_RRang.set(pid.calculate(RR_coder.getPosition(),controller.getLeftX()));
    // motor_RLang.set(pid.calculate(RL_coder.getPosition(),controller.getLeftX()));

    /*
     * TO DO
     * tune pid
     * lower sensitivity
     * field orientation (wait on gyro)
     */

  }

  private void drive(double x, double y, double z) {
    // ..A = predetermined tangential angle
    // ..Ang = desired wheel ang
    // ..X = controller + z component
    // ..Y = controller + z component

    // Front Right
    FRX = x + z * Math.cos(FRtangent);
    FRY = y + z * Math.sin(FRtangent);

    // Front left
    FLX = x + z * Math.cos(FLtangent);
    FLY = y + z * Math.sin(FLtangent);

    // Rear right
    RRX = x + z * Math.cos(RRtangent);
    RRY = y + z * Math.sin(RRtangent);

    // Rear left
    RLX = x + z * Math.cos(RLtangent);
    RLY = y + z * Math.sin(RLtangent);

    // final polar cords
    FRAng = Math.toDegrees(Math.atan2(FRY, FRX));
    FRMag = Math.sqrt((FRX * FRX) + (FRX * FRY));
    FLAng = Math.toDegrees(Math.atan2(FLY, FLX));
    FLMag = Math.sqrt((FLX * FLX) + (FLX * FLY));
    RRAng = Math.toDegrees(Math.atan2(RRY, RRX));
    RRMag = Math.sqrt((RRX * RRX) + (RRX * RRY));
    RLAng = Math.toDegrees(Math.atan2(RLY, RLX));
    RLMag = Math.sqrt((RLX * RLX) + (RLX * RLY));

    // set front right motors
    motor_FRang.set(pid.calculate(FR_coder.getPosition(), FRAng) + (-0.36 * FRMag));
    motor_FRmag.set(FRMag);

    // set front left motors
    motor_FLang.set(pid.calculate(FL_coder.getPosition(), FLAng) + (-0.36 * FLMag));
    motor_FLmag.set(FLMag);

    // set rear right motors
    motor_RRang.set(pid.calculate(RR_coder.getPosition(), RRAng) + (-0.36 * RRMag));
    motor_RRmag.set(RRMag);

    // set rear left motors
    motor_RLang.set(pid.calculate(RL_coder.getPosition(), RLAng) + (-0.36 * RLMag));
    motor_RLmag.set(RLMag);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}