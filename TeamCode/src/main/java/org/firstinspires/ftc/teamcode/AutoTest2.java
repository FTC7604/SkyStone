package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode;

@TeleOp(name = "AutoTest 2.0", group = "TeleOp")
public class AutoTest2 extends LinearOpMode {
    private RobotLinearOpMode robot;

    @Override
    public void runOpMode(){
        robot = new RobotLinearOpMode(this);

        waitForStart();

        telemetry.addLine("Positive Forward");
        telemetry.update();
        robot.moveByInches2(24, RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD);
        sleep(1000);

        telemetry.addLine("Positive Strafe");
        telemetry.update();
        robot.moveByInches2(24, RobotLinearOpMode.MOVEMENT_DIRECTION.STRAFE);
        sleep(1000);

        telemetry.addLine("Negative Forward");
        telemetry.update();
        robot.moveByInches2(-24, RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD);
        sleep(1000);

        telemetry.addLine("Negative Strafe");
        telemetry.update();
        robot.moveByInches2(-24, RobotLinearOpMode.MOVEMENT_DIRECTION.STRAFE);
        sleep(1000);
    }

}
