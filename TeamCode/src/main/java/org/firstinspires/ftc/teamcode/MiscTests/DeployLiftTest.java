package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.*;

/**  CONTAINS DEPLOY FUNCTION!  */

@TeleOp(name = "Deploy Test", group = "TeleOp")
@Disabled
public class DeployLiftTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private RobotLinearOpMode robot;

    @Override
    public void runOpMode() {
        robot = new RobotLinearOpMode(this, COLOR_SENSOR.NONE);
        robot.setAllMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setAllMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setAllMotorZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.initIMU();
        waitForStart();
        runtime.reset();

        robot.setLiftPower(-0.2);
        sleep(1000);
        robot.setLiftPower(0);
        robot.setArmPower(.2);
        sleep(750);
        robot.setArmPower(0);
        robot.setLiftZeroPowerProperty(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.setArmZeroPowerProperty(DcMotor.ZeroPowerBehavior.FLOAT);
        sleep(1000);
        robot.setLiftRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setArmRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.setLiftRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setArmRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
