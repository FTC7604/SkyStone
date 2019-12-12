/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import org.firstinspires.ftc.teamcode.PropertiesLoader;
import org.firstinspires.ftc.teamcode.Robot.*;

/**  CONTAINS ALL PRIOR VUFORIA TESTING  */

@TeleOp(name="Vuforia Test", group ="Autonomous")
@Disabled
public class vuforiaGang extends LinearOpMode {

    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.BACK;
    private static final String VUFORIA_KEY =
            "Adw59PP/////AAABmSngvZTKXktpu+nuzpPLAFUc6w406s2RYiPPvJaY9A1k2/zyXeM83mHvqT14sWp9QlghcCK1akohLb6SHQv4cXvD8AbeO1a9sRhhchx1X5eL6ttrRE5PH6g517XhKI0dvKsoeYhZu6k4ln6dacQOC11xv/AHSEi/VipxqOMXlNesBfv/jmCc48H6LTFTOHLVDEb9vkk7btw6StRcwle0PUdbCh5aPIkRI2pTh+0R1hY5FyGGrdyZltrBoUusodgwQW0sIai/V21YZGgKaN5QYZLOhO3Fv0ZhjWsnj52e/BivDb3RJyPF1loygTBADo6YZoki1S/oDzoqcP3VmjIaEIFr6RfIGrnVZtkVbjWZP+Zs";

    // Class Members
    private VuforiaTrackables targetsSkyStone = null;
    private VuforiaTrackable stoneTarget = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;

    //private RobotLinearOpMode robot;
    private PropertiesLoader propertiesLoader = new PropertiesLoader("Autonomous");
    private double BLOCK_POWER = propertiesLoader.getDoubleProperty("BLOCK_POWER");

    @Override public void runOpMode() {
        configureVuforia();
        //robot = new RobotLinearOpMode(this, COLOR_SENSOR.UNDER);
        waitForStart();
        targetsSkyStone.activate();

        while (!isStopRequested()) { // && !targetVisible
            targetVisible = false;

            if (((VuforiaTrackableDefaultListener)stoneTarget.getListener()).isVisible()) {
                telemetry.addData("Visible Target", stoneTarget.getName());
                targetVisible = true;
                OpenGLMatrix cameraPos = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getFtcCameraFromTarget();

                // attempting to get the pixel positions of the recognition
                if(cameraPos != null){
                    VectorF camPos = cameraPos.getTranslation();
                    telemetry.addLine("Pos: " + camPos.get(0) + " " + camPos.get(1));
                    telemetry.addLine("Dims: " + camPos.length());
                }

            }

            /*if (targetVisible) {
                robot.mecanumPowerDrive(0, 0, 0);
            }
            else {
                robot.mecanumPowerDrive(0, BLOCK_POWER, 0);
                telemetry.addData("Visible Target", "none");
            }*/

            telemetry.update();
        }

        targetsSkyStone.deactivate();
    }

    private void configureVuforia(){
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.useExtendedTracking = false;
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
    }

}

//class comment {
        /*while (!isStopRequested() && !targetVisible) {

            if (((VuforiaTrackableDefaultListener)stoneTarget.getListener()).isVisible()) {
                telemetry.addData("Visible Target", stoneTarget.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)stoneTarget.getListener()).getUpdatedRobotLocation();
                OpenGLMatrix cameraPos = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getFtcCameraFromTarget();

                // attempting to get the pixel positions of the recognition
                // may need to just use field positioning
                if(cameraPos != null){
                    VectorF camPos = cameraPos.getTranslation();
                    //telemetry.addLine("Pos: " + camPos.get(0) + " " + camPos.get(1));
                    //telemetry.addLine("Dims: " + camPos.length());
                }

                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }

            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                telemetry.addData("Visible Target", "SKYSTONE");
            }
            else {
                robot.mecanumPowerDrive(0, BLOCK_POWER, 0);
                telemetry.addData("Visible Target", "none");
            }

            telemetry.addData("Encoder", robot.getAverageForwardDriveTrainEncoder());
            //1900, 2500
            telemetry.update();
        }

        //31, 7
        //robot.moveByInches(31, RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD, false);
        //sleep(2000);

        /*if (((VuforiaTrackableDefaultListener)stoneTarget.getListener()).isVisible()) {
            telemetry.addData("Visible Target", stoneTarget.getName());
            targetVisible = true;
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {

        } else {
            telemetry.addData("Visible Target", "none");
        }

        telemetry.update();

        if(!targetVisible) {
            robot.moveByInches(7, RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD, false);
            sleep(1000);

            if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
                telemetry.addData("Visible Target", stoneTarget.getName());
                targetVisible = true;
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {

            } else {
                telemetry.addData("Visible Target", "none");
            }

            telemetry.update();
        }

        sleep(2000);

        targetsSkyStone.deactivate();*/

// Disable Tracking when we are done;*/
//}   //Contains old code