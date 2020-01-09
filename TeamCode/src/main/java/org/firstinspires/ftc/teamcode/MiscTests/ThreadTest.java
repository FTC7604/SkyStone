package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot.*;

import static org.firstinspires.ftc.teamcode.Robot.ThreadedRobotLinearOpMode.MOVEMENT_DIRECTION;

/**  TESTS THREADED DRIVE  */

@TeleOp(name = "Thread test", group = "Linear Opmode")
public class ThreadTest extends LinearOpMode {

    ThreadedRobotLinearOpMode robot;

    @Override
    public void runOpMode() {

        robot = new ThreadedRobotLinearOpMode(this);

        robot.setDriveTrainZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.setDriveTrainRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.setLiftZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.initIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //this is when the robot receives the 'play' command
        waitForStart();

        robot.threadedMoveByInches(0, MOVEMENT_DIRECTION.FORWARD, .05, .8);
        robot.threadedTurnToDegree(90);

        robot.waitForThreads();
    }
}
