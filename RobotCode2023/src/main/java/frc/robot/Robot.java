// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;

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

  PIDController pid = new PIDController(0.01, 0, 0);

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
  private double FRtangent = 175;

  private double FLX;
  private double FLY;
  private double FLtangent = 225;

  private double RRX;
  private double RRY;
  private double RRtangent = 0;

  private double RLX;
  private double RLY;
  private double RLtangent = 315;

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
    pid.setTolerance(1);
    pid.enableContinuousInput(0, 360);
    FR_coder.setPositionToAbsolute();
    FR_coder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    FL_coder.setPositionToAbsolute();
    FL_coder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    RR_coder.setPositionToAbsolute();
    RR_coder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    RL_coder.setPositionToAbsolute();
    RL_coder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // drive(controller.getRightX(), controller.getRightY(), controller.getLeftX());

    drive(1, 0, 0);
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

    // set front right motors
    pid.setSetpoint(Math.toDegrees(Math.atan2(FRY, FRX)) + 185);
    motor_FRang.set(pid.atSetpoint() ? 0 : pid.calculate(-FR_coder.getAbsolutePosition()));
    motor_FRmag.set((Math.hypot(FLY, FLX) / 10) + (0.36 * motor_FRang.get()));

    // set front left motors
    pid.setSetpoint(Math.toDegrees(Math.atan2(FLY, FLX)) + 100);
    motor_FLang.set(pid.atSetpoint() ? 0 : pid.calculate(-FL_coder.getAbsolutePosition()));
    motor_FLmag.set((Math.hypot(FLY, FLX) / 10) + (0.36 * motor_FLang.get()));

    // set rear right motors
    pid.setSetpoint(Math.toDegrees(Math.atan2(RRY, RRX)) + 120);
    motor_RRang.set(pid.atSetpoint() ? 0 : pid.calculate(-RR_coder.getAbsolutePosition()));
    motor_RRmag.set((Math.hypot(FLY, FLX) / 10) + (0.36 * motor_RRang.get()));

    // set rear left motors
    pid.setSetpoint(Math.toDegrees(Math.atan2(RLY, RLX)) + 165);
    motor_RLang.set(pid.atSetpoint() ? 0 : pid.calculate(-RL_coder.getAbsolutePosition()));
    motor_RLmag.set((Math.hypot(FLY, FLX) / 10) + (0.36 * motor_RLang.get()));

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