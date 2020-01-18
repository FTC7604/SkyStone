package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode;

@TeleOp(name = "AutoTest 2.0", group = "TeleOp")
@Disabled
public class AutoTest2 extends LinearOpMode {
    private RobotLinearOpMode robot;

    @Override
    public void runOpMode(){
        robot = new RobotLinearOpMode(this);

        waitForStart();

        telemetry.addLine("Positive Forward");
        telemetry.update();
        robot.moveByInchesFast(24, RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD);
        sleep(1000);

        telemetry.addLine("Positive Strafe");
        telemetry.update();
        robot.moveByInchesFast(24, RobotLinearOpMode.MOVEMENT_DIRECTION.STRAFE);
        sleep(1000);

        telemetry.addLine("Negative Forward");
        telemetry.update();
        robot.moveByInchesFast(-24, RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD);

        telemetry.addLine("Negative Strafe");
        telemetry.update();
        robot.moveByInchesFast(-24, RobotLinearOpMode.MOVEMENT_DIRECTION.STRAFE);

        telemetry.addLine("Positive Turn");
        telemetry.update();
        robot.turnToDegreeFast(90);

        telemetry.addLine("Negative Turn");
        telemetry.update();
        robot.turnToDegreeFast(-90);

        telemetry.addLine("Positive Turn");
        telemetry.update();
        robot.turnToDegreeFast(0);

    }

}
