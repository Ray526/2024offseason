package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.math.LinearRegression;
import frc.FSLib.math.PID;
import frc.FSLib.math.simplePID;
import frc.FSLib.util.AngularVelocity;
import frc.robot.Constants.UpperConstants;
import frc.robot.Constants.UpperStateMachine;
import frc.robot.Constants.UpperStateMachine.UpperState;
import frc.robot.Constants.MapConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.Upper;

public class TeleopUpper extends Command {

    private final Upper s_Upper;
    private final XboxController controller;

    private AngularVelocity intakeTarget;
    private AngularVelocity shooterTarget;
    private Rotation2d visionElbowTarget;

    // Constants such as camera and target height stored. Change per robot and goal!
    private final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(7.4);
    private final double TARGET_HEIGHT_METERS = Units.inchesToMeters(54);
    // Angle between horizontal and the camera.
    private final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(20);
    //chage the name accordingly
    PhotonCamera camera = new PhotonCamera("photonvision");

    private final PID elbowPID = new PID(
        UpperConstants.elbowKP,
        UpperConstants.elbowKI,
        UpperConstants.elbowKD,
        UpperConstants.elbowiWindup,
        UpperConstants.elbowiLimit
    );

    private final simplePID shooterPID = new simplePID(
        UpperConstants.shooterKP,
        UpperConstants.shooterKD
    );

    public TeleopUpper(Upper s_Upper, XboxController controller) {
        this.s_Upper = s_Upper;
        this.controller = controller;
        addRequirements(s_Upper);
    }

    @Override
    public void execute() {

        // var vision = camera.getLatestResult();

        // if (controller.getAButtonPressed() && vision.hasTargets()) {
        // SmartDashboard.putNumber("X", vision.getBestTarget().getBestCameraToTarget().getX());
        // double distanceToSpeaker = PhotonUtils.calculateDistanceToTargetMeters(
        //               CAMERA_HEIGHT_METERS,
        //               TARGET_HEIGHT_METERS,+
        //               CAMERA_PITCH_RADIANS,
        //               Units.degreesToRadians(vision.getBestTarget().getBestCameraToTarget().getX()));
        // visionElbowTarget = Rotation2d.fromRotations(
        // LinearRegression.calculate(MapConstants.DISTANCE_TO_ELBOW_AND_SHOOTER, distanceToSpeaker, 1));}
        // if (visionElbowTarget != null && controller.getAButtonPressed()) {
        // s_Upper.setElbow(-elbowPID.calculate(visionElbowTarget.getRotations() - s_Upper.getElbowRotation()));}

        if (controller.getYButtonPressed()) RobotConstants.upperState = RobotConstants.upperState == UpperState.GROUND ? UpperState.DEFAULT : UpperState.GROUND;
        if (controller.getXButtonPressed()) RobotConstants.upperState = RobotConstants.upperState == UpperState.AMP ? UpperState.DEFAULT : UpperState.AMP;
        if (controller.getAButtonPressed()) RobotConstants.upperState = RobotConstants.upperState == UpperState.SPEAKER ? UpperState.DEFAULT : UpperState.SPEAKER;
        if (controller.getBButtonPressed()) RobotConstants.upperState = RobotConstants.upperState == UpperState.BASE ? UpperState.DEFAULT : UpperState.BASE;
        if (controller.getRightBumperPressed()) RobotConstants.upperState = RobotConstants.upperState == UpperState.ENDGAME ? UpperState.DEFAULT : UpperState.ENDGAME;
        if (controller.getRightTriggerAxis() > 0.8) RobotConstants.upperState = UpperState.SHOOT;
        if (controller.getRightTriggerAxis() < 0.8 && RobotConstants.upperState == UpperState.SHOOT) RobotConstants.upperState = UpperState.DEFAULT;

        s_Upper.setElbow(-elbowPID.calculate(UpperStateMachine.elbowTarget.getRotations() - s_Upper.getElbowRotation()));
        if (RobotConstants.upperState == UpperState.GROUND && s_Upper.hasNote()&& RobotConstants.upperState != UpperState.SHOOT) {s_Upper.setIntake(0);} else {
        s_Upper.setIntake(UpperStateMachine.intakeTarget.getRevPM() / UpperConstants.INTAKE_MAX_RPM);}
        double output = LinearRegression.calculate(MapConstants.SHOOTER_RPM_TO_OUTPUT, UpperStateMachine.shooterTarget.getRevPM());
        s_Upper.setLeftShooter(output + shooterPID.calculate(s_Upper.getLeftShooterRPM(), UpperStateMachine.shooterTarget.getRevPM()));
        s_Upper.setRightShooter(output + shooterPID.calculate(s_Upper.getRightShooterRPM(), UpperStateMachine.shooterTarget.getRevPM()));

        SmartDashboard.putNumber("elbowTarget", UpperStateMachine.elbowTarget.getRotations());
    }
}
