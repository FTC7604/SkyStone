package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode;

import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.GRABBER_POSITION.DEFAULT;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.GRABBER_POSITION.DROPPING_SOON;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.GRABBER_POSITION.GRABBING;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.GRABBER_POSITION.READY;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.GRABBER_POSITION.STOWED;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.GRABBER_POSITION.GRABBING_SOON;


@TeleOp(name = "Grabber Test")
//@Disabled
public class GrabberTest extends LinearOpMode {
    private RobotLinearOpMode robot;

    @Override
    public void runOpMode() {
        double pos1 = 0;
        double pos2 = 0;

        robot = new RobotLinearOpMode(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.clearAll();

        while (!isStopRequested()) {

            if (gamepad1.a) {
                while (gamepad1.a) {
                }
                pos1 -= 0.1;
            }

            if (gamepad1.y) {
                while (gamepad1.y) {
                }
                pos1 += 0.1;
            }

            if (gamepad1.x) {
                while (gamepad1.x) {
                }
                pos2 -= 0.1;
            }

            if (gamepad1.b) {
                while (gamepad1.b) {
                }
                pos2 += 0.1;
            }

//            robot.setLeftGrabberPosition(pos1, pos2);

            if (gamepad2.a) robot.setLeftGrabberPosition(DEFAULT);
            if (gamepad2.b) robot.setLeftGrabberPosition(GRABBING);
            if (gamepad2.x) robot.setLeftGrabberPosition(STOWED);
            if (gamepad2.y) robot.setLeftGrabberPosition(READY);
            if (gamepad2.right_bumper) robot.setLeftGrabberPosition(GRABBING_SOON);
            if(gamepad2.right_trigger > 0) robot.setLeftGrabberPosition(DROPPING_SOON);

            if (gamepad2.dpad_down) robot.setRightGrabberPosition(DEFAULT);
            if (gamepad2.dpad_right) robot.setRightGrabberPosition(GRABBING);
            if (gamepad2.dpad_left) robot.setRightGrabberPosition(STOWED);
            if (gamepad2.dpad_up) robot.setRightGrabberPosition(READY);
            if (gamepad2.left_bumper) robot.setRightGrabberPosition(GRABBING_SOON);
            if(gamepad2.left_trigger > 0) robot.setRightGrabberPosition(DROPPING_SOON);

            //left
            //float pos: grabber @ 0.45, servo @ 0.6 (stowed)
            //grab pos: grabber @ 0.45, servo @ 0
            //start pos: grabber @ 1, servo @ 0
            //default pos: grabber @ 0.3, servo @ 0.8

            //right
            //float pos: grabber @ 0.7, servo @ 0.3 (stowed)
            //grab pos: grabber @ 0.7, servo @ 0.8
            //start pos: grabber @ 0, servo @ 0.8
            //default pos: grabber @ 0.85, servo @ -0.1

            telemetry.addData("Grabber position", pos1);
            telemetry.addData("Servo position", pos2);
            telemetry.update();
        }

    }

}
