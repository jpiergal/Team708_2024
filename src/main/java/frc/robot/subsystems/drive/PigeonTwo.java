package frc.robot.subsystems.drive;

// import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Pigeon2FeaturesConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class PigeonTwo {
    
    //Pigeon instance (ensure it is a singleton)
    private static PigeonTwo instance = null;

    /**
     * Method to get current instance of Pigeon2 (or establish one if one does not exist)
     * @return Pigeon2 Instance
     */
    public static PigeonTwo getInstance(){
        if(instance == null){
            instance = new PigeonTwo();
        }
        return instance;
    }

    //Pigeon2 object
    private Pigeon2 m_pigeon2;

    //Pigeon2 configuration settings
    private Pigeon2FeaturesConfigs config = new Pigeon2FeaturesConfigs();

    //Fault ErrorCode object
    // ErrorCode faults;

    //GravityVector object
    // private double[] a_gravityVector = new double[3];
    
    private PigeonTwo(){
        try{
            m_pigeon2 = new Pigeon2(4); //CHECK PORT AND CONSTRUCTOR
            //Sets the status frame period for two different periods.
            // m_pigeon2.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, 5, 10);
            // m_pigeon2.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 5, 10);
            
            //Default configs = MAKE CONSTANTS IN CONSTANTS FILE & DETERMINE VALUES
            config.DisableNoMotionCalibration = false;
            config.DisableTemperatureCompensation = false;
            config.EnableCompass = false; //true
            // config.MountPosePitch = 0;
            // config.MountPoseRoll = 0;
            // config.MountPoseYaw = 0;

            //Sets pigeon default mountings to values determined above
            m_pigeon2.getConfigurator().apply(config); //configAllSettings(config); //TODO

            //Gets gravity vectory and assigns it to a_gravityVector
            // m_pigeon2.getGravityVector(a_gravityVector); //TODO

            //Assigns faultLog to record errors
            // faults = m_pigeon2.getFaults(faultLog); //TODO
        }catch(Exception e){
            System.out.println("PIGEON INSTANTATION FAILED");
            // m_pigeon2.DestroyObject();// TODO
            e.printStackTrace();
        }
    }

    // public boolean isGood(){
    //     return true;
    // }

    public Rotation2d getPitch(){
        double pitch = m_pigeon2.getPitch().getValue();
        return Rotation2d.fromDegrees(pitch);
    }

    public Rotation2d getAngle(){
        double yaw = m_pigeon2.getYaw().getValue(); //TODO NEGATED YAW FIXES AUTOS
        return Rotation2d.fromDegrees(yaw);
    }

    public Rotation2d getRoll(){
        double roll = m_pigeon2.getRoll().getValue(); //COMPARE TO getYPR();
        return Rotation2d.fromDegrees(roll);
    }

    public double getRateX(){
        double[] ypr = new double[3];
        m_pigeon2.getRawMagneticFieldX();
        return ypr[0];
    }

    // public double getRateY(){
    //     double[] ypr = new double[3];
    //     m_pigeon2.getRawGyro(ypr);
    //     return ypr[1];
    // }

    // public double getRateZ(){
    //     double[] ypr = new double[3];
    //     m_pigeon2.getRawGyro(ypr);
    //     return ypr[2];
    // }

    // public double get360Angle(){
    //     double[] ypr = new double[3];
    //     m_pigeon2.getYawPitchRoll(ypr);
    //     return ypr[0] % 360;
    // }

    public StatusCode setAngle(double degrees){
        return m_pigeon2.setYaw(degrees);
    }

    public StatusCode reset(){
        return setAngle(0.0);
    }

    //CODE RELATED TO ERROR MANAGEMENT//

    public boolean getAccelError(){
        return m_pigeon2.getFault_BootupAccelerometer().getValue();
    }

    public boolean getBootIntoMotionError(){
        return m_pigeon2.getFault_BootIntoMotion().getValue();
    }

    public boolean getGyroError(){
        return m_pigeon2.getFault_BootupGyroscope().getValue();
    }

    public boolean getHardwareError(){
        return m_pigeon2.getFault_Hardware().getValue();
    }

    public boolean getMagnetometerError(){
        return m_pigeon2.getFault_BootupMagnetometer().getValue();
    }

    public boolean getMotionDriverError(){
        return m_pigeon2.getFault_BootIntoMotion().getValue();
    }

    public boolean getFault_SaturatedAccelerometer(){
        return m_pigeon2.getFault_SaturatedAccelerometer().getValue();
    }

    public boolean getFault_SaturatedMagnetometer(){
        return m_pigeon2.getFault_SaturatedMagnetometer().getValue();
    }

    public boolean getFault_SaturatedGyroscope(){
        return m_pigeon2.getFault_SaturatedGyroscope().getValue();
    }

    // public boolean getVoltageError(){
    //     return m_pigeon2.getFault_Undervoltage();
    // }

    public void printPigeonErrorLog(){
        System.out.println("Accel: " + getAccelError());
        System.out.println("BootIntoMotion: " + getBootIntoMotionError());
        System.out.println("Gyro: " + getGyroError());
        System.out.println("Hardware: " + getHardwareError());
        System.out.println("Magnetometer: " + getMagnetometerError());
        System.out.println("MotionDriver: " + getMotionDriverError());
        System.out.println("SaturatedAccel: " + getFault_SaturatedAccelerometer());
        System.out.println("SaturatedMag: " + getFault_SaturatedMagnetometer());
        System.out.println("SaturatedRotVel: " + getFault_SaturatedGyroscope());
        // System.out.println("Voltage: " + getVoltageError());
    }

}