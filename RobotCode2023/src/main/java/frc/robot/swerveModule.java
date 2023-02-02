/*
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class swerveModule extends Robot {
    WPI_TalonFX driveMotor;
    WPI_TalonFX steerMotor;
    CANCoder moduleEncoder;

    /**
     * @param drive
     * @param steer
     * @param encoder
     
    public swerveModule(WPI_TalonFX drive,WPI_TalonFX steer,CANCoder encoder){
      driveMotor = drive;
      steerMotor = steer;
      moduleEncoder = encoder;
    }
    /**
     * @param velocity desired velocity, coordanates range [-1.0.707106,1,0.707106]
     
    void moduleSetVelocity(vector velocity){
      double x = velocity.getX();
      double y = velocity.getY();
      double encoderAng = moduleEncoder.getAbsolutePosition();

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
      steerMotor.set(pid.calculate(encoderAng, setang) * angSpeedMax);
      driveMotor.set((setmag * magSpeedMax) + (swerveRatio * steerMotor.get()));
    }
    /**
     * 
     
    vector moduleGetVelocity(){
      double FRmoduleSpeed = moduleSpeed(driveMotor, steerMotor);
      double FRmoduleAng = Math.toRadians(gyroYaw + -FR_coderPosition);
      velocityX = FRmoduleSpeed * Math.cos(FRmoduleAng);
      velocityY = FRmoduleSpeed * Math.sin(FRmoduleAng);
      vector resultant = new vector("cartesian",velocityX,velocityY);
      return resultant;
    }
    /**
   * @return wheel speed is measured in Feet/20ms(periodic cycle) because its
   *         pronounced soccer
   * @param driveMotor Driving motor of the swerve module
   * @param steerMotor Steering motor of the swerve module
   
    double moduleGetSpeed() {
    return (//
      (//
        (getMotorVelocity(driveMotor) * drive1stStageRatio) + // speed of 3rd drive gear
        (getMotorVelocity(steerMotor) * steeringGearRatio) // steer speed at steering shaft
      )// relative speed of 4th drive gear
        * (drive2ndStageRatio) * (drive3rdStageRatio) * // wheel speed
        (WheelRadiusFt)// convert radians per cycle to feet per cycle
    );
  }
}
*/