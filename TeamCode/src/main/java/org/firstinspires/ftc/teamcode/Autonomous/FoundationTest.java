package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DWAIAutonomous;
import org.firstinspires.ftc.teamcode.IO.PropertiesLoader;
import org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode;
import org.firstinspires.ftc.teamcode.Robot.StageSwitchingPipeline;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.GRABBER_POSITION.DEFAULT;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.GRABBER_POSITION.DROPPING_SOON;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.GRABBER_POSITION.GRABBING;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.GRABBER_POSITION.GRABBING_SOON;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.GRABBER_POSITION.READY;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.GRABBER_POSITION.STOWED;

@TeleOp(name = "Foundation Tester")
public class FoundationTest extends LinearOpMode {
    private PropertiesLoader propertiesLoader = new PropertiesLoader("Autonomous");
    private double BRIDGE_Y_POSITION = propertiesLoader.getDoubleProperty("BRIDGE_Y_POSITION");
    private double INITIAL_FOUNDATION_X_POSITION = propertiesLoader.getDoubleProperty("INITIAL_FOUNDATION_X_POSITION");
    private double FOUNDATION_Y_POSITION = propertiesLoader.getDoubleProperty("FOUNDATION_Y_POSITION");
    private double DEPOT_Y_POSITION = propertiesLoader.getDoubleProperty("DEPOT_Y_POSITION");

    private long GRAB_DELAY = propertiesLoader.getLongProperty("GRAB_DELAY");

    private long DROP_BLOCK_DELAY = propertiesLoader.getLongProperty("DROP_BLOCK_DELAY");
    private long DROP_RETURN_DELAY = propertiesLoader.getLongProperty("DROP_RETURN_DELAY");
    private long GET_GRAB_DELAY = propertiesLoader.getLongProperty("GET_GRAB_DELAY");
    private long GET_BLOCK_DELAY = propertiesLoader.getLongProperty("GET_BLOCK_DELAY");
    private long GET_STOW_DELAY = propertiesLoader.getLongProperty("GET_STOW_DELAY");

    private double OPEN_LATCH_SERVO_POSITION = propertiesLoader.getDoubleProperty("OPEN_LATCH_SERVO_POSITION");
    private double CLOSE_LATCH_SERVO_POSITION = propertiesLoader.getDoubleProperty("CLOSE_LATCH_SERVO_POSITION");

    private boolean PAUSE_STEPS = propertiesLoader.getBooleanProperty("PAUSE_STEPS");
    private boolean DEPLOY_LIFTER = propertiesLoader.getBooleanProperty("DEPLOY_LIFTER");
    private double LATERAL_MULTIPLIER = propertiesLoader.getDoubleProperty("LATERAL_MULTIPLIER");

    private SampleMecanumDriveREV drive;
    private RobotLinearOpMode robot;

    //TODO: incorporate that into the past trajectory %5
    private void strafeTo(double x_pos, double y_pos, double additionalStrafe) {
        //check if turn necessary
        /*double turnAngle = drive.getPoseEstimate().getHeading();

        if(turnAngle > Math.PI){
            turnAngle -= 2 * Math.PI;
        }

        drive.turnSync(-turnAngle);*/

        if (additionalStrafe != 0) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeTo(new Vector2d(x_pos, y_pos))
                            .strafeRight(additionalStrafe)
                            .build()
            );
        } else {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeTo(new Vector2d(x_pos, y_pos))
                            .build()
            );
        }

    }

    @Override
    public void runOpMode() {
        robot = new RobotLinearOpMode(this);
        drive = new SampleMecanumDriveREV(hardwareMap);

        //Additional setup
        robot.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        telemetry.addLine("Initialized!");
        telemetry.update();

        waitForStart();

        //Some roadrunner stuff
        robot.setLeftGrabberPosition(DEFAULT);
        robot.setRightGrabberPosition(DEFAULT);
        robot.setLatchPosition(OPEN_LATCH_SERVO_POSITION);

        dragFoundation();
        telemetry.addLine("Done!");
        telemetry.update();

        //AutoTransitioner.transitionOnStop(opMode, "Skystone Main Teleop", alliance);
    }

    private void dragFoundation() {
        drive.setPoseEstimate(new Pose2d(15, 63, Math.toRadians(90)));

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(50, FOUNDATION_Y_POSITION, Math.toRadians(90)))
                        .build()
        );

        while (!robot.getFoundationSensorPressed()) {
            robot.mecanumPowerDrive(0, -0.3, 0);
            drive.updatePoseEstimate();
        }

        robot.mecanumPowerDrive(0, 0, 0);
        robot.setLatchPosition(CLOSE_LATCH_SERVO_POSITION);

        sleep(100);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(12, BRIDGE_Y_POSITION, Math.toRadians(180)))
                        .build()
        );

        double turnAngle = drive.getPoseEstimate().getHeading() - Math.PI;

        if (turnAngle > Math.PI) {
            turnAngle -= 2 * Math.PI;
        }

        drive.turnSync(-turnAngle);
        robot.setLatchPosition(OPEN_LATCH_SERVO_POSITION);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(5)
                        .splineTo(new Pose2d(0, BRIDGE_Y_POSITION, Math.toRadians(180)))
                        .build()
        );
    }

}
