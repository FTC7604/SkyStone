package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.*;

@TeleOp(name = "Intake Test", group = "TeleOp")
public class IntakeTest extends LinearOpMode {
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

        robot.mecanumPowerDrive(0, 0.2, 0);
        robot.setIntakePower(1);

        while(!robot.getBlockSensorPressed() && opModeIsActive()){

        }

        robot.mecanumPowerDrive(0, 0, 0);
        robot.setIntakePower(0);
    }
}
