package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.SensorValues;

import java.util.concurrent.LinkedBlockingQueue;

import static java.lang.Thread.sleep;

class SensorRunnable implements Runnable {

    //necessary so that the thread can access the sensors
    private Robot robot;
    //necessary so taht it can be passed into the queue
    private SensorValues values = new SensorValues();
    //the queue itself that moves data from the thread to the
    LinkedBlockingQueue <SensorValues> sensorToMotors;

    private boolean stop;


    SensorRunnable(Robot robot, LinkedBlockingQueue<SensorValues> sensorToMotors) {
        this.robot = robot;
        this.sensorToMotors = sensorToMotors;

    }

    public void run() {
        while(!stop) {
            try {
                setSensorValues();

                //eliminates unnecessary data
                if (sensorToMotors.size() > 2) sensorToMotors.poll();
                //sends it off
                sensorToMotors.add(values);

            }
            catch (Exception e) {

            }
        }
    }

    public void stop(){
        stop = true;
    }

    public void setSensorValues(){
        //sets the values from the robot
        values.setxRotation(robot.getBothIMUAngle()[0]);
        values.setyRotation(robot.getBothIMUAngle()[1]);
        values.setzRotation(robot.getBothIMUAngle()[2]);


    }
}
