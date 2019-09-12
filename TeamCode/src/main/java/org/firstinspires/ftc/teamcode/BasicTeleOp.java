package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
public class BasicTeleOp extends LinearOpMode {

    private DcMotor lf;
    private DcMotor rf;
    private DcMotor lb;
    private DcMotor rb;

    private DcMotor lIntake;
    private DcMotor rIntake;

    public void initHardware(){
        telemetry.addData("Status", "Setting up");
        telemetry.update();

        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");

        lf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);

        lIntake = hardwareMap.get(DcMotor.class, "lIntake");
        rIntake = hardwareMap.get(DcMotor.class, "rIntake");
    }

    @Override
    public void runOpMode(){
        initHardware();
        waitForStart();

        while(opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double intakePower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            leftPower  = gamepad1.left_stick_y ;
            rightPower = gamepad1.right_stick_y ;

            // Send calculated power to wheels
            lf.setPower(leftPower);
            rf.setPower(rightPower);
            lb.setPower(leftPower);
            rb.setPower(rightPower);

            if(gamepad1.right_bumper){
                intakePower = 1;
            } else if(gamepad1.left_bumper){
                intakePower = -1;
            } else{
                intakePower = 0;
            }

            rIntake.setPower(intakePower);
            lIntake.setPower(-intakePower);

            // Show the status and wheel power.
            telemetry.addData("Status", "Running");
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }

    }

}
