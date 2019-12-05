package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.*;

/**  TESTING FOR SKYSTONE FOUNDATION DURING AUTONOMOUS  */

@TeleOp(name = "Foundation Test", group = "TeleOp")
public class FoundationTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private RobotLinearOpMode robot;
    private PropertiesLoader propertiesLoader = new PropertiesLoader("Autonomous");

    @Override
    public void runOpMode() {
        robot = new RobotLinearOpMode(this, COLOR_SENSOR.NONE);
        robot.setAllMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setAllMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setAllMotorZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.initIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        robot.openLatch();
        robot.mecanumPowerDrive(0, -0.3, 0);

        while(!robot.getFoundationSensorPressed() && opModeIsActive()){

        }

        robot.mecanumPowerDrive(0, 0, 0);
        robot.closeLatch();
    }

}
