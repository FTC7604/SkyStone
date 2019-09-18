package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.IMUControl;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.SensorValues;

import java.util.concurrent.LinkedBlockingQueue;

@TeleOp(name="Field Centric", group="Linear Opmode")
public class testFieldCentric extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    IMUControl imuControl =  new IMUControl();

    double[] driveTrainController = new double[3];

    /*The descriptions of the robot in terms of sensor values, position, encoder position
    and so on. All of the sensor values should be in here */
    SensorValues sensorValues  = new SensorValues();

    /*The Queue that safely passes the sensor values from a core that is getting them, into
    the loop, this should run on its own core, enabling the robot to run faster. */
    LinkedBlockingQueue <SensorValues> sensorValuesQueue = new LinkedBlockingQueue();


    @Override
    public void runOpMode() {
        /*The robot objects is all of the motors, and sensors that are within the robot,
        essentially the code framework of the robot*/
        Robot robot = new Robot(this);

        /*Creates the threads, its own processor, to get the sensor values*/
        SensorRunnable sensorRunnable = new SensorRunnable(robot, sensorValuesQueue);
        Thread sensorThread = new Thread(sensorRunnable);

        //robot.calibrateIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //this is when the robot receives the 'play' command
        waitForStart();

        //starts up the thread and begins the runtime
        sensorThread.start();
        runtime.reset();

        robot.initIMU();

        while (opModeIsActive()) {

            //sets up the condidtion for the drivetrain
            driveTrainController[0] = /*HumanController.humanController*/(gamepad1.left_stick_x);
            driveTrainController[1] = /*HumanController.humanController*/(-gamepad1.left_stick_y);
            driveTrainController[2] = /*HumanController.humanController*/(-gamepad1.right_stick_x);

            //reset the IMU if a is pressed
            //if(gamepad1.a)robot.calibrateIMU();

            telemetry.addLine("The wait is after we poll the darn thing");
            telemetry.update();

            //gets the sensor values from the queue
            sensorValues = getSensorValues(sensorValues);

            telemetry.addLine("The wait is calculating the sensor values");
            telemetry.update();

            //takes the imputs, and changes the to compensate for the opposite of the detected angles
            imuControl.compensate(driveTrainController, - robot.getBothIMUAngle()[2]);

            //sends this to the motors.
            robot.mecPowerDrive(driveTrainController);

            //some telemetry
            telemetry.addData("X Rotation", sensorValues.getxRotation());
            telemetry.addData("Y Rotation", sensorValues.getyRotation());
            telemetry.addData("Z Rotation", sensorValues.getzRotation());
            telemetry.update();
        }

        sensorRunnable.stop();
    }

    public SensorValues getSensorValues(SensorValues sensorValues){
        /*This method is a little bit confusing, essentially it takes the
        data off of the queue and returns it from the method, if an error is
        caught it returns the error then it returns the existing sensor values.
         */

        try {
            if(sensorValuesQueue.poll() != null) {
                return sensorValuesQueue.take();
            }
            else return sensorValues;
        }
        catch (InterruptedException e) {
            telemetry.addData("Exception:", e);
            telemetry.update();
            sleep(30000);

            return sensorValues;
        }
    }
}

