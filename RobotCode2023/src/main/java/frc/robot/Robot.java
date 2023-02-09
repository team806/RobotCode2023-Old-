// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
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
  // controller
  private final XboxController controller = new XboxController(0);
  // gyro
  public static final ADIS16470_IMU IMU = new ADIS16470_IMU();
  // PIDs
  PIDController pid = new PIDController(0.02, 0, 0);
  PIDController ballancePID = new PIDController(0.02, 0, 0);
  PIDController driveXPID = new PIDController(0.02, 0, 0);
  PIDController driveYPID = new PIDController(0.02, 0, 0);
  PIDController driveAngPID = new PIDController(0.02, 0, 0);
  // Talons
  private final WPI_TalonFX motor_FRang = new WPI_TalonFX(3);
  private final WPI_TalonFX motor_FRmag = new WPI_TalonFX(2);
  private final WPI_TalonFX motor_FLang = new WPI_TalonFX(8);
  private final WPI_TalonFX motor_FLmag = new WPI_TalonFX(4);
  private final WPI_TalonFX motor_RRang = new WPI_TalonFX(1);
  private final WPI_TalonFX motor_RRmag = new WPI_TalonFX(6);
  private final WPI_TalonFX motor_RLang = new WPI_TalonFX(5);
  private final WPI_TalonFX motor_RLmag = new WPI_TalonFX(7);
  // CANCoders
  CANCoder FR_coder = new CANCoder(9);
  CANCoder FL_coder = new CANCoder(11);
  CANCoder RR_coder = new CANCoder(10);
  CANCoder RL_coder = new CANCoder(12);
  //modules
  //swerveModule FR_module = new swerveModule(motor_FRmag,motor_FRang,FR_coder);
  //swerveModule FL_module = new swerveModule(motor_FLmag,motor_FLang,FL_coder);
  //swerveModule RR_module = new swerveModule(motor_RRmag,motor_RRang,RR_coder);
  //swerveModule RL_module = new swerveModule(motor_RLmag,motor_RLang,RL_coder);
  // encoder position values
  double FR_coderPosition;
  double FL_coderPosition;
  double RR_coderPosition;
  double RL_coderPosition;
  // constants
  protected double swerveRatio = -0.36;
  // motor speed maximums
  protected double angSpeedMax = 0.3;
  protected double magSpeedMax = 1 - (angSpeedMax * -swerveRatio);
  // x,y position of the robot in feet and degrees
  double robotX = 0;
  double robotY = 0;
  double gyroYaw;
  // clockwise gyro yaw
  double CWgyroYaw;
  // velocity of robot in feet per 20ms
  double velocityX;
  double velocityY;
  //
  boolean onChargeStation;
  // modifiers for controller inputs (not for drive or module drive methods)
  double controllerMagMax = 0.8;
  double controllerMagPow = 3;
  // target 2D pose values for
  double targetRobotAng;
  double targetRobotY;
  double targetRobotX;

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

    pid.enableContinuousInput(0, 360);
    FR_coder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    FR_coder.configSensorDirection(true);
    FR_coder.configMagnetOffset(102);
    FL_coder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    FL_coder.configSensorDirection(true);
    FL_coder.configMagnetOffset(0);
    RR_coder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    RR_coder.configSensorDirection(true);
    RR_coder.configMagnetOffset(-70);
    RL_coder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    RL_coder.configSensorDirection(true);
    RL_coder.configMagnetOffset(-28);
    IMU.calibrate();
    IMU.setYawAxis(IMUAxis.kZ);

    SmartDashboard.putNumber("angSpeedMax", angSpeedMax);
    SmartDashboard.putNumber("magSpeedMax", magSpeedMax);

    SmartDashboard.putNumber("controller Mag Max", controllerMagMax);
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
    // CCW = positive, centered to NORTH
    gyroYaw = IMU.getAngle();
    // CW = positive, centered to EAST
    CWgyroYaw = -(gyroYaw + 90);
    // CW = positive, centered to EAST
    FR_coderPosition = FR_coder.getAbsolutePosition();
    FL_coderPosition = FL_coder.getAbsolutePosition();
    RR_coderPosition = RR_coder.getAbsolutePosition();
    RL_coderPosition = RL_coder.getAbsolutePosition();
    /*
     * double velocityX = ((wheelSpeed(motor_FRmag) * Math.cos(CWgyroYaw +
     * FR_coderPosition)) +
     * (wheelSpeed(motor_FLmag) * Math.cos(CWgyroYaw + FL_coderPosition)) +
     * (wheelSpeed(motor_RRmag) * Math.cos(CWgyroYaw + RR_coderPosition)) +
     * (wheelSpeed(motor_RLmag) * Math.cos(CWgyroYaw + RL_coderPosition))) / 4;
     * double velocityY = ((wheelSpeed(motor_FRmag) * Math.sin(CWgyroYaw +
     * FR_coderPosition)) +
     * (wheelSpeed(motor_FLmag) * Math.sin(CWgyroYaw + FL_coderPosition)) +
     * (wheelSpeed(motor_RRmag) * Math.sin(CWgyroYaw + RR_coderPosition)) +
     * (wheelSpeed(motor_RLmag) * Math.sin(CWgyroYaw + RL_coderPosition))) / 4;
     * 
     * robotX += velocityX;
     * robotY += velocityY;
     * 
     * SmartDashboard.putNumber("X velocity", velocityX);
     * SmartDashboard.putNumber("Y velocity", velocityY);
     * SmartDashboard.putNumber("X", robotX);
     * SmartDashboard.putNumber("Y", robotY);
     * SmartDashboard.putNumber("Yaw", gyroYaw);
     */
    /*
     * SmartDashboard.putNumber("FR codeer ang", FR_coderPosition);
     * SmartDashboard.putNumber("FL codeer ang", FL_coderPosition);
     * SmartDashboard.putNumber("RR codeer ang", RR_coderPosition);
     * SmartDashboard.putNumber("RL codeer ang", RL_coderPosition);
     * 
     * SmartDashboard.putNumber("FR ang motor", motor_FRang.get());
     * SmartDashboard.putNumber("FL ang motor", motor_FLang.get());
     * SmartDashboard.putNumber("RR ang motor", motor_RRang.get());
     * SmartDashboard.putNumber("RL ang motor", motor_RLang.get());
     * SmartDashboard.putNumber("FR mag motor", motor_FRmag.get());
     * SmartDashboard.putNumber("FL mag motor", motor_FLmag.get());
     * SmartDashboard.putNumber("RR mag motor", motor_RRmag.get());
     * SmartDashboard.putNumber("RL mag motor", motor_RLmag.get());
     */
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

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    controllerMagMax = SmartDashboard.getNumber("controller Mag Max", controllerMagMax);

    drive(controller.getLeftX(), controller.getLeftY(), Math.pow(controller.getRightX(), controllerMagPow));

    if (controller.getAButtonPressed()) {
      IMU.reset();
    }
  }

  /**
   * @return Drives the robot in specified direction, rotates input according to
   *         gyroYaw
   * @param x one to negitive one value for driving left or right respectivly
   * @param y one to negitive one value for driving back or forward respectivly
   * @param z one to negitive one value for driving counter clockwise or clockwise
   *          respectivly
   */
  private void drive(double x, double y, double z) {
    double controlerAng = Math.atan2(y, x) + Math.toRadians(gyroYaw + 90);// rotate by gyro for field
    double controlermag = Math.pow(MathUtil.clamp(Math.hypot(x, y), -1, 1), controllerMagPow) * controllerMagMax;
    x = Math.cos(controlerAng) * controlermag;
    y = Math.sin(controlerAng) * controlermag;

    moduleDrive(motor_FRang, motor_FRmag, FR_coderPosition, x + (z * -0.707106), y + (z * 0.707106));
    moduleDrive(motor_FLang, motor_FLmag, FL_coderPosition, x + (z * 0.707106), y + (z * 0.707106));
    moduleDrive(motor_RRang, motor_RRmag, RR_coderPosition, x + (z * -0.707106), y + (z * -0.707106));
    moduleDrive(motor_RLang, motor_RLmag, RL_coderPosition, x + (z * 0.707106), y + (z * -0.707106));

    SmartDashboard.putNumber("final x", x);
    SmartDashboard.putNumber("final y", y);
  }

  /**
   * @return Drive a swerve module acording to an x and y value. NOT field
   *         oriended
   * @param angMotor   the steering motor of the respective module
   * @param magMotor   the steering motor of the respective module
   * @param encoderAng the encoder ang of the respective module
   * @param x          one to negitive one value for driving left or right
   *                   respectivly
   * @param y          one to negitive one value for driving back or forward
   *                   respectivly
   */
  void moduleDrive(WPI_TalonFX angMotor, WPI_TalonFX magMotor, double encoderAng, double x, double y) {
    double targetAng = Math.toDegrees(Math.atan2(y, x));
    double targetMag = Math.hypot(y, x) / (1 + Math.hypot(0.707106, 0.707106));// scaled to range [-1,1]
    double setang;
    double setmag;

    if (distance(encoderAng, targetAng) > 90) {
      setang = targetAng + 180;
      setmag = -targetMag;
    } else {
      setang = targetAng;
      setmag = targetMag;
    }

    angMotor.set(pid.calculate(encoderAng, setang) * angSpeedMax);
    magMotor.set((setmag * magSpeedMax) + (swerveRatio * angMotor.get()));
  }

  public static double distance(double alpha, double beta) {
    double phi = Math.abs(beta - alpha) % 360; // This is either the distance or 360 - distance
    double distance = phi > 180 ? 360 - phi : phi;
    return distance;
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
    // onChargeStation = false;
    robotX = 0;
    robotY = 0;
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // test ballance code
    // controller.setRumble(RumbleType.kLeftRumble, 1);
    /*
     * if (onChargeStation) {
     * drive(0, ballancePID.calculate(gyroPitch, 0), 0);
     * } else {
     * drive(0, 0.10, 0);
     * if (gyroPitch > 10) {
     * onChargeStation = true;
     * }
     * }
     */

    //drive(driveXPID.calculate(robotX),driveYPID.calculate(robotY),driveAngPID.calculate(gyroYaw));

<<<<<<< HEAD
    drive(0.0, controller.getLeftY(), 0.0);
    if (controller.getAButtonPressed()) {IMU.reset();}

    double FRmoduleSpeed = moduleSpeed(motor_FLmag, motor_FLang);
    double FRmoduleAng = Math.toRadians(gyroYaw + -FR_coderPosition);
=======
    drive(controller.getLeftX(), 0.0, 0.0);

    if (controller.getAButtonPressed()) {
      IMU.reset();
    }

    double FRmoduleSpeed = moduleSpeed(motor_FRmag, motor_FRang);
    double FRmoduleAng = Math.toRadians(-(gyroYaw + FR_coderPosition));
>>>>>>> parent of da7acc3 (Update Robot.java)
    velocityX = FRmoduleSpeed * Math.cos(FRmoduleAng);
    velocityY = FRmoduleSpeed * Math.sin(FRmoduleAng);
    robotX += velocityX;
    robotY += velocityY;

    SmartDashboard.putNumber("X ft/s", velocityX * 50);
    SmartDashboard.putNumber("Y ft/s", velocityY * 50);
    SmartDashboard.putNumber("X", robotX);
    SmartDashboard.putNumber("Y", robotY);
    SmartDashboard.putNumber("Yaw", gyroYaw);
    SmartDashboard.putNumber("fr drive motor speed", motor_FLmag.getSelectedSensorVelocity());
  }
<<<<<<< HEAD
  /**
   * @return wheel speed is measured in Feet/20ms(periodic cycle) because its
   *         pronounced soccer
   * @param driveMotor Driving motor of the swerve module
   * @param steerMotor Steering motor of the swerve module
   */
  double moduleSpeed(WPI_TalonFX driveMotor, WPI_TalonFX steerMotor) {
    return (//
      (//
        (getMotorVelocity(driveMotor) * drive1stStageRatio) + // speed of 3rd drive gear
        (getMotorVelocity(steerMotor) * steeringGearRatio) // steer speed at steering shaft
      )// relative speed of 4th drive gear
        * (drive2ndStageRatio) * (drive3rdStageRatio) * // wheel speed
        (WheelRadiusFt)// convert radians per cycle to feet per cycle
    );
  }
  /**
   * @return Returns motor velocity in radians per 20ms(one periodic cycle)
   */
  double getMotorVelocity(WPI_TalonFX motor) {
    return motor.getSelectedSensorVelocity() * motorUnitsToRadsSeconds;
=======

  // notice, wheel speed is measured in Feet/20ms because its pronounced soccer
  double moduleSpeed(WPI_TalonFX driveMotor, WPI_TalonFX steerMotor) {
    return ((driveMotor.getSelectedSensorVelocity() / 2867.2) + (steerMotor.getSelectedSensorVelocity() / 21948.5714))
        * ((14 / 25) * Math.PI);
>>>>>>> parent of da7acc3 (Update Robot.java)
  }
}