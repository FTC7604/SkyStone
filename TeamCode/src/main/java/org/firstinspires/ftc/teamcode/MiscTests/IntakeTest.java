package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.*;

/**  TESTING FOR SKYSTONE INTAKE DURING AUTONOMOUS  */

@TeleOp(name = "Intake Test", group = "TeleOp")
@Disabled
public class IntakeTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private RobotLinearOpMode robot;

    @Override
    public void runOpMode() {
        robot = new RobotLinearOpMode(this);
        robot.setAllMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setAllMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setAllMotorZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.initIMU();
        waitForStart();
        runtime.reset();

        robot.mecanumPowerDrive(0, 0.2, 0);
        robot.setIntakePower(1);

        while(robot.getBlockSensorNotPressed() && opModeIsActive()){

        }

        robot.mecanumPowerDrive(0, 0, 0);
        robot.setIntakePower(0);
    }
}
