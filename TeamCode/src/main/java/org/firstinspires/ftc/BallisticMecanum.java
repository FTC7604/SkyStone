/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//Ignore this
package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;

@TeleOp(name="Ballistic Mecanum", group="Linear Opmode")
//@Disabled


public class BallisticMecanum extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack    = null;
    private DcMotor rightBack   = null;

    private BNO055IMU imu = null;


    ////////IN DEGREES
    final double TOP_LIMIT = 0;//has to be greater than the bottom limit, thus positive power goes to top. if this isn't done, stuff will break
    final double BOTTOM_LIMIT = 0;//the bottom of the lifter, can be negative
    final double THRESHOLD = 90;//the deceleration threshold. stays the same for both the top and the botttom
    final double FLOOR_POWER = 0.05;//the minimum power for a motor to run
    final double EXP_POWER = 1;//to exponentially increase the curve
    final double MAX_POWER = 0.65;

    private BallisticMotionProfile TurnProfile = new BallisticMotionProfile(TOP_LIMIT, BOTTOM_LIMIT, THRESHOLD, FLOOR_POWER, EXP_POWER, MAX_POWER);//what we will be using to slow it down\

    private BallisticMotionProfile DriveProfile = new BallisticMotionProfile(0, 0, 800, 0.05, 1, 0.75);//for driving

    double adjustedMotorPower = 0;//applied to all motors left and right

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFront  = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        double currentAngle = 0;//of angle
        double currentPosition = 0;//of encoder

        setMotorBehaviors();

        waitForStart();
        runtime.reset();

        startIMU();

        while (opModeIsActive()) {

            if (gamepad1.dpad_left) {

                startIMU();

                double startAngle = imu.getAngularOrientation().firstAngle;//IMU.get some rotation stuff
                double neededAngle = startAngle + 90;//current angle + 90
                currentAngle = startAngle;

                telemetry.addData("starting", startAngle);
                telemetry.addData("needed", neededAngle);

                while ((currentAngle < neededAngle) && opModeIsActive()) {//will run until we get there
                    currentAngle = imu.getAngularOrientation().firstAngle;////IMU something

                    adjustedMotorPower = TurnProfile.RunToPositionWithAccel(startAngle, currentAngle, neededAngle);//get a rotation

                    leftFront.setPower(-adjustedMotorPower);
                    leftBack.setPower(-adjustedMotorPower);
                    rightFront.setPower(adjustedMotorPower);
                    rightBack.setPower(adjustedMotorPower);
                }
            }
            else if (gamepad1.dpad_right) {

                startIMU();

                double startAngle = imu.getAngularOrientation().firstAngle;//IMU.get some rotation stuff
                double neededAngle = startAngle - 90;//current angle + 90
                currentAngle = startAngle;

                telemetry.addData("starting", startAngle);
                telemetry.addData("needed", neededAngle);

                while ((currentAngle > neededAngle) && opModeIsActive()) {//will run until we get there
                    currentAngle = imu.getAngularOrientation().firstAngle;////IMU something

                    adjustedMotorPower = TurnProfile.RunToPositionWithAccel(startAngle, currentAngle, neededAngle);//get a rotation

                    leftFront.setPower(-adjustedMotorPower);
                    leftBack.setPower(-adjustedMotorPower);
                    rightFront.setPower(adjustedMotorPower);
                    rightBack.setPower(adjustedMotorPower);
                }
            }
            else if (gamepad1.dpad_up) {///see if we can do an encoder drive as well

                double startPosition = leftFront.getCurrentPosition();

                double neededInches = 20;//change this

                double neededPosition = startPosition + (neededInches*4000/69);

                while ((leftFront.getCurrentPosition() < neededPosition)&&opModeIsActive()) {
                    currentPosition = leftFront.getCurrentPosition();

                    adjustedMotorPower = DriveProfile.RunToPositionWithAccel(startPosition, currentPosition, neededPosition);

                    leftFront.setPower(adjustedMotorPower);
                    leftBack.setPower(adjustedMotorPower);
                    rightFront.setPower(adjustedMotorPower);
                    rightBack.setPower(adjustedMotorPower);
                }
            }
            else if (gamepad1.dpad_down){

                double startPosition = leftFront.getCurrentPosition();

                double neededInches = 20;//change this

                double neededPosition = startPosition - (neededInches*4000/69);


                while ((leftFront.getCurrentPosition() > neededPosition)&&opModeIsActive()) {
                    currentPosition = leftFront.getCurrentPosition();

                    adjustedMotorPower = DriveProfile.RunToPositionWithAccel(startPosition, currentPosition, neededPosition);

                    leftFront.setPower(adjustedMotorPower);
                    leftBack.setPower(adjustedMotorPower);
                    rightFront.setPower(adjustedMotorPower);
                    rightBack.setPower(adjustedMotorPower);
                }
            } else {
                RunDrive();
            }

            telemetry.update();
        }
    }

    private void RunDrive () {
        double y = (((-gamepad1.left_stick_y)*(abs(-gamepad1.left_stick_y))+((-gamepad1.right_stick_y)*(abs(-gamepad1.right_stick_y))))/2);
        double x = (((-gamepad1.left_stick_x)*(abs(-gamepad1.left_stick_x))+((-gamepad1.right_stick_x)*(abs(-gamepad1.right_stick_x))))/2);
        double turnVal = (((-gamepad1.left_stick_y)-(-gamepad1.right_stick_y))/2);

        telemetry.addData("position", leftBack.getCurrentPosition());

        leftFront.setPower(y-x+turnVal);
        leftBack.setPower(y+x+turnVal);
        rightFront.setPower(y+x-turnVal);
        rightBack.setPower(y-x-turnVal);
    }

    private void setMotorBehaviors () {
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //creates the parameters that the IMU uses.
    private void startIMU() {
        // We are expecting the IMU to be attached to an I2C port on a Core Device Interface Module and named "imu".
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        parameters.angleUnit      = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }

}
