package org.firstinspires.ftc.teamcode.MiscTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import static java.lang.Thread.sleep;

@TeleOp(group = "TeleOp", name = "Trajectory Tester")
public class RoadRunnerTest extends LinearOpMode {
    private RobotLinearOpMode robot;
    private volatile boolean flag = false;

    @Override
    public void runOpMode(){
        robot = new RobotLinearOpMode(this);
        SampleMecanumDriveREV drive = new SampleMecanumDriveREV(hardwareMap);

        telemetry.addLine("Initialized!");
        telemetry.update();

        waitForStart();

        robot.setLeftGrabberPosition(RobotLinearOpMode.GRABBER_POSITION.DEFAULT);
        robot.setRightGrabberPosition(RobotLinearOpMode.GRABBER_POSITION.DEFAULT);
        drive.setPoseEstimate(new Pose2d(-33, 63, Math.toRadians(90)));

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(12)
                        .build()
        );

        Thread deploy = new Thread(() -> {
            startDeploy();
            flag = true;
        });

        deploy.start();

        drive.turnSync(0);

        robot.setRightGrabberPosition(RobotLinearOpMode.GRABBER_POSITION.READY);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-20, 33.6, 0))
                        .build()
        );

        robot.setRightGrabberPosition(RobotLinearOpMode.GRABBER_POSITION.GRABBING);
        sleep(500);
        robot.setRightGrabberPosition(RobotLinearOpMode.GRABBER_POSITION.STOWED);
        sleep(500);

        while(!flag);

        /*drive.followTrajectory(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(0, 40, 0))
                        .splineTo(new Pose2d(45, 35, 0))
                        .build()
        );

        while(drive.isBusy() && opModeIsActive()){
            drive.update();
        }*/

        telemetry.addLine("Done!");
        telemetry.update();

        //X diffs
        //POS 1: -20, 33.6
        //POS 2: -28, 33.6
        //POS 3: -36, 33.6

        //BRIDGE: 0, 40 < go here between blocks
        //FOUNDATION: 45, 35

        //X diffs
        //POS 4: -44, 33.6
        //POS 5: -52, 33.6
        //POS 6: -60, 33.6

        //turn to 90 at end
        //move to 60, 63
        //park at 0, 38

    }

    private void startDeploy(){
        robot.setLiftPower(-0.5);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        robot.setLiftPower(0);

        robot.setArmPower(.2);
        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        robot.setArmPower(0);

        robot.setLiftPower(0.2);
        robot.setArmPower(-.2);

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        robot.setLiftPower(0);
        robot.setArmPower(0);

        robot.setLiftZeroPowerProperty(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.setArmZeroPowerProperty(DcMotor.ZeroPowerBehavior.FLOAT);

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        robot.setLiftRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setArmRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.setLiftRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setArmRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
