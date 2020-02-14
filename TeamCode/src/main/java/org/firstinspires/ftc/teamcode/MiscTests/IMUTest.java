package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode;


@TeleOp(name="IMU Test")
public class IMUTest extends LinearOpMode{
    private RobotLinearOpMode robot;

    @Override
    public void runOpMode(){
        robot = new RobotLinearOpMode(this);

        DcMotorEx rf = (DcMotorEx) hardwareMap.get(DcMotor.class, "lf");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        while(!isStopRequested()){
            double[] angles2 = robot.getRev2IMUAngle();
            double[] angles10 = robot.getRev10IMUAngle();

            telemetry.addData("Hub:", " Rev 2");
            telemetry.addData("First angle", angles2[2]);
            telemetry.addData("Second angle", angles2[0]);
            telemetry.addData("Third angle", angles2[1]);

            telemetry.addData("Hub:", " Rev 10");
            telemetry.addData("First angle", angles10[2]);
            telemetry.addData("Second angle", angles10[0]);
            telemetry.addData("Third angle", angles10[1]);

            telemetry.update();
        }

    }

}
