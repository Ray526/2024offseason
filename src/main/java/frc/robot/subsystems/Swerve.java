// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;

public class Swerve extends SubsystemBase {
  private static Swerve mInstance;

  private final Pigeon2 gyro = new Pigeon2(SwerveConstants.pigeon1, RobotConstants.canbusName);

  public SwerveModule[] mSwerveMods = new SwerveModule[] {
    new SwerveModule(0, SwerveConstants.Mod0.constants),
    new SwerveModule(1, SwerveConstants.Mod1.constants),
    new SwerveModule(2, SwerveConstants.Mod2.constants),
    new SwerveModule(3, SwerveConstants.Mod3.constants)
  };

  private SwerveDriveKinematics kinematics = SwerveConstants.kinematics;

  private SwerveDriveOdometry swerveOdometry = new SwerveDriveOdometry(kinematics, getGyroscopeRotation(), getPositions());

  private PoseEstimatorSubsystem poseEstimater;

  private Field2d m_field = new Field2d();

  private double AngularVel;

  private Pose2d[] mModulePoses = new Pose2d[4];

  private Pose2d mRobotPose = new Pose2d();

  ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

  // private final StructArrayPublisher<SwerveModuleState> publisher;

  public Swerve() {
    // ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    gyro.setYaw(0);

    // ChassisSpeeds speeds = new ChassisSpeeds(1.0, 3.0, 1.5);

    // SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    // SwerveModuleState LF = moduleStates[0];
    // SwerveModuleState RF = moduleStates[1];
    // SwerveModuleState LR = moduleStates[2];
    // SwerveModuleState RR = moduleStates[3];

    // var LFOptimized = SwerveModuleState.optimize(LF, new Rotation2d())

    // publisher = NetworkTableInstance.getDefault()
    //   .getStructArrayTopic("SwerveStates", SwerveModuleState.struct).publish();

    //   var LFState = new SwerveModuleState(23.43, Rotation2d.fromDegrees(-140.19));
    //   var RFState = new SwerveModuleState(23.43, Rotation2d.fromDegrees(-39.81));
    //   var LRState = new SwerveModuleState(54.08, Rotation2d.fromDegrees(-109.44));
    //   var RRState = new SwerveModuleState(54.08, Rotation2d.fromDegrees(-70.56));

    //   ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(LFState, RFState, LRState, RRState);

    //   double forward = chassisSpeeds.vxMetersPerSecond;
    //   double sideways = chassisSpeeds.vyMetersPerSecond;
    //   double angular = chassisSpeeds.omegaRadiansPerSecond;

    AutoBuilder.configureHolonomic(
      this::getOdometryPose, // Robot pose supplier (getOdometryPose or getPoseEstimated)
      this::setPose,  // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      SwerveConstants.pathFollowerConfig,
      () -> {
          if (DriverStation.getAlliance().isPresent()) return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
          return false;
      },
      this
    );
    }
  

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getGyroscopeRotation())
                    : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxModuleSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxModuleSpeed);
    for (SwerveModule mod : mSwerveMods) { 
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getOdometryPose() {
    return swerveOdometry.getPoseMeters();
  }

  public Pose2d getEstimatedPose() {
    if (poseEstimater != null) {
      System.out.println("using Pose Estmater");
    return poseEstimater.getCurrentPose();}
    else {
      System.out.println("using Odometry");
      return swerveOdometry.getPoseMeters();}
  }

  public void setPose(Pose2d pose) {
    swerveOdometry.resetPosition(getGyroscopeRotation(), getPositions(), pose);
  }

  public void resetOdometryPose(Pose2d pose) {
    if (poseEstimater != null) {
      System.out.println("pose estimater reset");
      poseEstimater.resetPosition(getGyroscopeRotation(), getModulePositions(), pose);
    } else {
      System.out.println("reset odometry");
      swerveOdometry.resetPosition(getGyroscopeRotation(), getPositions(), pose);
    }
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getOdometryPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getStates());
  }

  // return Robot Relative Speeds
  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        Constants.SwerveConstants.kinematics.toChassisSpeeds(getStates()),
        getGyroscopeRotation());

    return chassisSpeeds;
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = mSwerveMods[0].getPosition();
    positions[1] = mSwerveMods[1].getPosition();
    positions[2] = mSwerveMods[2].getPosition();
    positions[3] = mSwerveMods[3].getPosition();
    return positions;
  }

  // for pose estimater
  public SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(mSwerveMods).map(module -> module.getPosition()).toArray(SwerveModulePosition[]::new);
  }

  // for pose estimater
  public Rotation2d getGyroscopeRotation() {
    return gyro.getRotation2d();
  }

  public void setYaw(Rotation2d pos) {
    gyro.setYaw(pos.getDegrees());
  }

  public static Swerve getInstance() {
    if (mInstance == null) {
        mInstance = new Swerve();
    }
    return mInstance;
  }

  public void stop() {
    driveFieldRelative(new ChassisSpeeds());
  }

  private void updateSwervePoses() {
    if(getOdometryPose() != null) mRobotPose = getOdometryPose();
    else mRobotPose = new Pose2d();
    for (int i = 0; i < mModulePoses.length; i++) {
        Translation2d updatedPosition = Constants.SwerveConstants.swerveModuleLocations[i]
                .rotateBy(mRobotPose.getRotation()).plus(mRobotPose.getTranslation());
        Rotation2d updatedRotation = getStates()[i].angle.plus(mRobotPose.getRotation());
        if(getStates()[i].speedMetersPerSecond < 0.0) {
            updatedRotation = updatedRotation.plus(Rotation2d.fromDegrees(180));;
        }
        mModulePoses[i] = new Pose2d(updatedPosition, updatedRotation);
    }
  }

  @Override
  public void periodic() {
    updateSwervePoses();
    m_field.setRobotPose(mRobotPose);
    m_field.getObject("Swerve Modules Pose").setPoses(mModulePoses);
    
    swerveOdometry.update(getGyroscopeRotation(), getPositions());
    AngularVel = Math.max(gyro.getAngularVelocityZWorld().getValueAsDouble(), AngularVel);
    // m_field.setRobotPose(getOdometryPose());
    SmartDashboard.putData("Field", m_field);
    // tab.add("Yaw", getGyroscopeRotation())
    //   .withSize(2, 4)
    //   .withPosition(0, 0);
    // tab.add("AngularVel", AngularVel)
    //   .withSize(2, 4)
    //   .withPosition(2, 0);
    // tab.add("OdometryX", getOdometryPose().getX())
    //   .withSize(2, 4)
    //   .withPosition(4, 0);
    // tab.add("OdometryY", getOdometryPose().getY())
    //   .withSize(2, 4)
    //   .withPosition(6, 0);
    // tab.add("OdometryRotation", getOdometryPose().getRotation().getRotations());

    // publisher.set(new SwerveModuleState[] {
    //   LFState,
    //   RFState,
    //   LRState,
    //   RRState
    // });
    // SmartDashboard.putDate("States", getStates());
  }
}
