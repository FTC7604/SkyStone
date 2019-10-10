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


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode;

@Autonomous(name = "Ballistic Mecanum Auto", group = "Linear Opmode")
//@Disabled

public class BallisticMecanumAuto extends LinearOpMode {

    RobotLinearOpMode robotLinearOpMode;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robotLinearOpMode = new RobotLinearOpMode(this);


        robotLinearOpMode.setDriveTrainZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);
        robotLinearOpMode.setDriveTrainRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotLinearOpMode.setDriveTrainRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robotLinearOpMode.initIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        //robot.calibrateIMU();

        int startAngleRev2 = (int) robotLinearOpMode.getRev2IMUAngle()[2];
        int startAngleRev10 = (int) robotLinearOpMode.getRev10IMUAngle()[2];

        robotLinearOpMode.stopMotorsAndWait(.5);

        robotLinearOpMode.turnByDegree(90);

        robotLinearOpMode.stopMotorsAndWait(.5);

        int endAngleRev2 = (int) robotLinearOpMode.getRev2IMUAngle()[2];
        int endAngleRev10 = (int) robotLinearOpMode.getRev10IMUAngle()[2];

        double startPosition = robotLinearOpMode.getAverageDriveTrainEncoder(RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD);
        robotLinearOpMode.moveByInches(24, RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD);
        double endPosition = robotLinearOpMode.getAverageDriveTrainEncoder(RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD);

        telemetry.addData("Start Angle REV 2: ", startAngleRev2);
        telemetry.addData("End Angle REV 2: ", endAngleRev2);
        telemetry.addData("Different REV 2: ", endAngleRev2 - startAngleRev2);
        telemetry.addLine(" ");
        telemetry.addData("Start Angle REV 10: ", startAngleRev10);
        telemetry.addData("End Angle REV 10: ", endAngleRev10);
        telemetry.addData("Different REV 10: ", endAngleRev10 - startAngleRev10);
//        telemetry.addData("StartPosition: ", startPosition * 69/4000);
//        telemetry.addData("endPosition: ", endPosition * 69/4000);
//        telemetry.addData("differentPosition: ", (startPosition - endPosition) * 69/4000);
        telemetry.update();

        sleep(15000);
    }


}
