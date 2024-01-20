// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.*;
import frc.robot.utilities.FieldRelativeAccel;
import frc.robot.utilities.FieldRelativeSpeed;

  /**
   * Implements a swerve Drivetrain Subsystem for the Robot
   */
  public class Drivetrain extends SubsystemBase {

  //Create the PIDController for the Keep Angle PID
  private final PIDController m_keepAnglePID = new PIDController(DriveConstants.kKeepAnglePID[0],
    DriveConstants.kKeepAnglePID[1],DriveConstants.kKeepAnglePID[2]);
  
  private double keepAngle = 0.0;       //Double to store the current target keepAngle in radians
  private double timeSinceRot = 0.0;    //Double to store the time since last rotation command
  private double lastRotTime = 0.0;     //Double to store the time of the last rotation command
  private double timeSinceDrive = 0.0;  //Double to store the time since last translation command
  private double lastDriveTime = 0.0;   //Double to store the time of the last translation command

  private double radius = 1;//0.450;

  private boolean m_readyToShoot = false;

  private boolean fieldOrient = true;

  private final SlewRateLimiter m_slewX = new SlewRateLimiter(12.0);
  private final SlewRateLimiter m_slewY = new SlewRateLimiter(12.0);
  private final SlewRateLimiter m_slewRot = new SlewRateLimiter(20.0);

  private FieldRelativeSpeed m_fieldRelVel = new FieldRelativeSpeed();
  private FieldRelativeSpeed m_lastFieldRelVel = new FieldRelativeSpeed();
  private FieldRelativeAccel m_fieldRelAccel = new FieldRelativeAccel();

  private final Timer keepAngleTimer = new Timer(); //Creates timer used in the perform keep angle function

  //Creates a swerveModule object for the front left swerve module feeding in parameters from the constants file
  private final SwerveModule m_frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort, DriveConstants.kFrontLeftTurningEncoderPort,
      DriveConstants.kFrontLeftOffset, DriveConstants.kFrontLeftTuningVals);

  //Creates a swerveModule object for the front right swerve module feeding in parameters from the constants file
  private final SwerveModule m_frontRight = new SwerveModule(DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort, DriveConstants.kFrontRightTurningEncoderPort,
      DriveConstants.kFrontRightOffset, DriveConstants.kFrontRightTuningVals);

  //Creates a swerveModule object for the back left swerve module feeding in parameters from the constants file
  private final SwerveModule m_backLeft = new SwerveModule(DriveConstants.kRearLeftDriveMotorPort,
      DriveConstants.kBackLeftTurningMotorPort, DriveConstants.kBackLeftTurningEncoderPort,
      DriveConstants.kBackLeftOffset, DriveConstants.kBackLeftTuningVals);

  //Creates a swerveModule object for the back right swerve module feeding in parameters from the constants file
  private final SwerveModule m_backRight = new SwerveModule(DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kBackRightTurningMotorPort, DriveConstants.kBackRightTurningEncoderPort,
      DriveConstants.kBackRightOffset, DriveConstants.kBackRightTuningVals);
  
  //Get pigeon gyro instance
  private static PigeonTwo pigeon = PigeonTwo.getInstance();

  //Creates Odometry object to store the pose of the robot
  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, pigeon.getAngle(), getModulePositions());

  private final SwerveDriveOdometry m_autoOdometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, pigeon.getAngle(), getModulePositions());

  ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
    AutoConstants.kThetaControllerConstraints);

  private final Field2d m_field;
    /**
   * Constructs a Drivetrain and resets the Gyro and Keep Angle parameters
   */
  public Drivetrain() {
    keepAngleTimer.reset();
    keepAngleTimer.start();
    m_keepAnglePID.enableContinuousInput(-Math.PI, Math.PI);
    pigeon.reset();
    // m_odometry.resetPosition(pigeon.getAngle().times(-1.0), getModulePositions(), new Pose2d()); //JNP 
    m_odometry.resetPosition(pigeon.getAngle().times(1.0), getModulePositions(), new Pose2d()); //JNP 
    CommandScheduler.getInstance().registerSubsystem(this);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    m_field = new Field2d();

    // SmartDashboard.putData("Field", m_field);
    }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean keepAngle) {
    // SmartDashboard.putBoolean("fieldRelative", fieldRelative);
    // SmartDashboard.putBoolean("keepAngle", keepAngle);

    // if(keepAngle){
    //   rot = performKeepAngle(xSpeed,ySpeed,rot); //Calls the keep angle function to update the keep angle or rotate depending on driver input
    // }
    
    xSpeed = m_slewX.calculate(xSpeed);
    ySpeed = m_slewY.calculate(ySpeed);
    rot = m_slewRot.calculate(rot);
    
    // SmartDashboard.putNumber("xSpeed Commanded", xSpeed);
    // SmartDashboard.putNumber("ySpeed Commanded", ySpeed);
    // SmartDashboard.putNumber("rot Commanded", rot);

    //creates an array of the desired swerve module states based on driver command and if the commands are field relative or not
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, pigeon.getAngle())
            : new ChassisSpeeds(xSpeed * pigeon.getAngle().getCos() + ySpeed * pigeon.getAngle().getSin(), 
            (-xSpeed * pigeon.getAngle().getSin() + ySpeed * pigeon.getAngle().getCos()) - (radius * rot), 
            rot));

    //normalize wheel speeds so all individual states are scaled to achievable velocities
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSec); 

    setModuleStates(swerveModuleStates);
  }

  @Override
  public void periodic(){
      m_fieldRelVel = new FieldRelativeSpeed(getChassisSpeed(), getGyro());
      m_fieldRelAccel = new FieldRelativeAccel(m_fieldRelVel, m_lastFieldRelVel, GlobalConstants.kLoopTime);
      //m_fieldRelJerk = new FieldRelativeJerk(m_fieldRelAccel, m_lastFieldRelAccel, GlobalConstants.kLoopTime);
      //m_lastFieldRelAccel = m_fieldRelAccel;
      m_lastFieldRelVel = m_fieldRelVel;

      // SmartDashboard.putNumber("RobotSpeedX", getChassisSpeed().vxMetersPerSecond);
      // SmartDashboard.putNumber("RobotSpeedY", getChassisSpeed().vyMetersPerSecond);
      // SmartDashboard.putNumber("RobotOmega", getChassisSpeed().omegaRadiansPerSecond);

      // SmartDashboard.putNumber("Robot pitch degrees", pigeon.getPitch().getDegrees());
      // SmartDashboard.putNumber("Robot roll degrees", pigeon.getRoll().getDegrees());

      // SmartDashboard.putNumber("Accel X", m_fieldRelAccel.ax);
      // SmartDashboard.putNumber("Accel Y", m_fieldRelAccel.ay);
      // SmartDashboard.putNumber("Alpha", m_fieldRelAccel.alpha);

        SmartDashboard.putNumber("Front Left Encoder", m_frontLeft.getTurnEncoder());
        SmartDashboard.putNumber("Front Right Encoder", m_frontRight.getTurnEncoder());
        SmartDashboard.putNumber("Back Left Encoder", m_backLeft.getTurnEncoder());
        SmartDashboard.putNumber("Back Right Encoder", m_backRight.getTurnEncoder());

        SmartDashboard.putNumber("Balance Angle", pigeon.getRoll().getDegrees());

        //Update swerve drive odometry periodically so robot pose can be tracked
        updateOdometry();    

        //Calls get pose function which sends the Pose information to the SmartDashboard
        getPose();

        m_field.getRobotObject().setPose(getPose());
  }

  public void setFieldOrient(boolean fieldOrient){
    this.fieldOrient = fieldOrient;
  }

  public boolean getFieldOrient(){
    return fieldOrient;
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSec);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModuleState[] getModuleStates(){
    return new SwerveModuleState[]{
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_backLeft.getState(),
      m_backRight.getState()
    };
  }

  public void stop(){
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  /**
   * Updates odometry for the swerve drivetrain. This should be called
   * once per loop to minimize error.
   */  
  public void updateOdometry() {
    m_odometry.update(pigeon.getAngle(), getModulePositions());
  }

  public void updateAutoOdometry() {
    m_autoOdometry.update(pigeon.getAngle(), getModulePositions());
  }

  /**
   * Function to retrieve latest robot gyro angle.
   * @return Rotation2d object containing Gyro angle
   */  
  public Rotation2d getGyro() {
    return pigeon.getAngle();
  }

  public double getGyroDegrees() {
    return pigeon.getAngle().getDegrees();
  }

  public FieldRelativeSpeed getFieldRelativeSpeed(){
    return m_fieldRelVel;
  }

  public FieldRelativeAccel getFieldRelativeAccel(){
    return m_fieldRelAccel;
  }

    /**
   * Function created to retreieve and push the robot pose to the SmartDashboard for diagnostics
   * @return Pose2d object containing the X and Y position and the heading of the robot.
   */  
  public Pose2d getPose() {
    Pose2d pose = m_odometry.getPoseMeters();
    Translation2d position = pose.getTranslation();
    //Rotation2d heading = getGyro();
    SmartDashboard.putNumber("Robot X", position.getX());
    SmartDashboard.putNumber("Robot Y", position.getY());
    SmartDashboard.putNumber("Robot Gyro", getGyro().getRadians());
    return m_odometry.getPoseMeters();
  }

  public Pose2d getAutoPose() {
    updateAutoOdometry();
    Pose2d pose = m_autoOdometry.getPoseMeters();
    Translation2d position = pose.getTranslation();
    SmartDashboard.putNumber("Auto X", position.getX());
    SmartDashboard.putNumber("Auto Y", position.getY());
    return m_autoOdometry.getPoseMeters();
  }

  /**
   * Resets the odometry and gyro to the specified pose.
   *
   * @param pose in which to set the odometry and gyro.
   */
  public void resetOdometry(Pose2d pose) {
    pigeon.reset();
    pigeon.setAngle(pose.getRotation().getDegrees());
    keepAngle = getGyro().getRadians();

    m_odometry.resetPosition(pigeon.getAngle().times(-1.0), getModulePositions(), pose);
    m_autoOdometry.resetPosition(pigeon.getAngle().times(-1.0), getModulePositions(), pose);
  }

  public void setPose(Pose2d pose){
    m_odometry.resetPosition(pigeon.getAngle().times(-1.0), getModulePositions(), pose);
        keepAngle = getGyro().getRadians();
  }


    /**
   * Resets the gyro to the given angle
   * 
   * @param angle the angle of the robot to reset to
   */
  public void resetOdometry(Rotation2d angle) {
    Pose2d pose = new Pose2d(getPose().getTranslation(), angle);
    pigeon.reset();
    pigeon.setAngle(angle.getDegrees());
    keepAngle = getGyro().getRadians();
    m_odometry.resetPosition(pigeon.getAngle().times(-1.0), getModulePositions(), pose);  }

  /**
   * Converts the 4 swerve module states into a chassisSpeed by making use of the swerve drive kinematics.
   * @return ChassisSpeeds object containing robot X, Y, and Angular velocity 
   */  
  public ChassisSpeeds getChassisSpeed(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(),
    m_backRight.getState());
  }

  /**
   * Keep angle function is performed to combat drivetrain drift without the need of constant "micro-adjustments" from the driver.
   * A PIDController is used to attempt to maintain the robot heading to the keepAngle value. This value is updated when the robot 
   * is rotated manually by the driver input
   * @return rotation command in radians/s
   * @param xSpeed is the input drive X speed command
   * @param ySpeed is the input drive Y speed command
   * @param rot is the input drive rotation speed command
   */  
  private double performKeepAngle(double xSpeed, double ySpeed, double rot){
    double output = rot; //Output should be set to the input rot command unless the Keep Angle PID is called
    if(Math.abs(rot) >= DriveConstants.kMinRotationCommandRadPerSec){  //If the driver commands the robot to rotate set the last rotate time to the current time
      lastRotTime = keepAngleTimer.get();
    }
    if( Math.abs(xSpeed) >= DriveConstants.kMinTranslationCommandMetersPerSec  
          || Math.abs(ySpeed) >= DriveConstants.kMinTranslationCommandMetersPerSec){ //if driver commands robot to translate set the last drive time to the current time
      lastDriveTime = keepAngleTimer.get();
    }
    timeSinceRot = keepAngleTimer.get()-lastRotTime;      //update variable to the current time - the last rotate time
    timeSinceDrive = keepAngleTimer.get()-lastDriveTime;  //update variable to the current time - the last drive time
    if(timeSinceRot < 0.5){                               //Update keepAngle up until 0.5s after rotate command stops to allow rotation move to finish
      keepAngle = getGyro().getRadians();
    }   //JNP anythingn here to tweek rotate speed??
    else if(Math.abs(rot) < DriveConstants.kMinRotationCommandRadPerSec && timeSinceDrive < 0.25){ //Run Keep angle pid until 0.75s after drive command stops to combat decel drift
      output = m_keepAnglePID.calculate(getGyro().getRadians(), keepAngle);               //Set output command to the result of the Keep Angle PID 
    }
    return output;
  }

  public void updateKeepAngle(){
    keepAngle = getGyro().getRadians();
  }

  public void setReadytoShoot(){
    m_readyToShoot = true;
  }

  public boolean isReadyToShoot(){
    return m_readyToShoot;
  }

  private SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[] {m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(),
      m_backRight.getPosition()};
  }

  public void invertDrive(){
    m_frontLeft.invertDrive();
    m_frontRight.invertDrive();
    m_backLeft.invertDrive();
    m_backRight.invertDrive();
  }

  public void sendToDashboard() {
  }
  
}