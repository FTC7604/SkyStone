package org.firstinspires.ftc.teamcode.Robot;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/*

Config:

rightFrontDriveMotor -> "rf"
leftFrontDriveMotor -> "lf"
rightBackDriveMotor ->  "rb"
leftBackDriveMotor -> "lb"

rightIntakeMotor -> "ri"
leftIntakeMotor -> "ri"

armMotor -> "ax"
liftMotor -> "lx"

leftLatchServo -> "ll"
rightLatchServo -> "rl"
blockGrabberServo -> "bg"


blockIntakeTouchSensor -> "bt"


 */

public class Robot {
    private BNO055IMU imu1 = null, imu2 = null;
    DcMotorEx rightFrontDriveMotor;
    DcMotorEx leftFrontDriveMotor;
    DcMotorEx rightBackDriveMotor;
    DcMotorEx leftBackDriveMotor;

    DcMotorEx rightIntakeMotor;
    DcMotorEx leftIntakeMotor;

    DcMotorEx armMotor;
    DcMotorEx liftMotor;

    Servo leftLatchServo;
    Servo rightLatchServo;
    Servo blockGrabberServo;

    //private ColorSensor colorSensor;
    //private DistanceSensor distanceSensor;

    private DigitalChannel blockIntakeTouchSensor;

    private HardwareMap hardwareMap;

    public Robot(OpMode opMode) {

        this.hardwareMap = opMode.hardwareMap;
        mapHardware();
    }

    private void mapHardware() {
        driveTrainHardwareMap();

        intakeHardwareMap();

        armHardwareMap();
        liftHardwareMap();

        latchHardwareMap();
        blockGrabberHardwareMap();

        imuHardwareMap();
        blockIntakeTouchSensorHardwareMap();
    }

    private void driveTrainHardwareMap() {
        rightFrontDriveMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "lf");
        leftFrontDriveMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rf");
        rightBackDriveMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "lb");
        leftBackDriveMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rb");

        //dir
        rightFrontDriveMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftFrontDriveMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDriveMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDriveMotor.setDirection(DcMotorEx.Direction.FORWARD);

    }

    public void setDriveTrainZeroPowerProperty(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        leftFrontDriveMotor.setZeroPowerBehavior(zeroPowerBehavior);
        rightFrontDriveMotor.setZeroPowerBehavior(zeroPowerBehavior);
        leftBackDriveMotor.setZeroPowerBehavior(zeroPowerBehavior);
        rightBackDriveMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setDriveTrainRunMode(DcMotor.RunMode runMode) {
        leftFrontDriveMotor.setMode(runMode);
        rightFrontDriveMotor.setMode(runMode);
        leftBackDriveMotor.setMode(runMode);
        rightBackDriveMotor.setMode(runMode);
    }

    private void intakeHardwareMap() {
        //intake
        rightIntakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "ri");
        leftIntakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "li");

        //sets the direction of the intake
        rightIntakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftIntakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void setIntakeZeroPowerProperty(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        //so that it floats by at 0, could also be stop
        rightIntakeMotor.setZeroPowerBehavior(zeroPowerBehavior);
        leftIntakeMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setIntakeRunMode(DcMotor.RunMode runMode) {
        rightIntakeMotor.setMode(runMode);
        leftIntakeMotor.setMode(runMode);
    }

    private void armHardwareMap() {
        //intake
        armMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "ax");
    }

    public void setArmZeroPowerProperty(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        //so that it floats by at 0, could also be stop
        armMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setArmRunMode(DcMotor.RunMode runMode) {
        armMotor.setMode(runMode);
    }

    private void liftHardwareMap() {
        //intake
        liftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "lx");

        liftMotor.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void setLiftZeroPowerProperty(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        //so that it floats by at 0, could also be stop
        liftMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setLiftRunMode(DcMotor.RunMode runMode) {
        liftMotor.setMode(runMode);
    }

    private void latchHardwareMap() {
        leftLatchServo = hardwareMap.get(Servo.class, "ll");
        rightLatchServo = hardwareMap.get(Servo.class, "rl");

        leftLatchServo.setDirection(Servo.Direction.FORWARD);
        rightLatchServo.setDirection(Servo.Direction.REVERSE);
    }

    private void blockGrabberHardwareMap() {
        blockGrabberServo = hardwareMap.get(Servo.class, "bg");
    }


    //methods that will serve us in the future, so we don't need to think about anything
    private void imuHardwareMap() {
        imu1 = hardwareMap.get(BNO055IMU.class, "imu");
        imu2 = hardwareMap.get(BNO055IMU.class, "imu 1");
    }

    //    private void colorSensorHardwareMap(){
//        //gets the right sensor
//        colorSensor = hardwareMap.get(ColorSensor.class, "rcs");
//        distanceSensor = hardwareMap.get(DistanceSensor.class, "rds");
//    }

    private void blockIntakeTouchSensorHardwareMap() {
        blockIntakeTouchSensor = hardwareMap.get(DigitalChannel.class, "bt");
        blockIntakeTouchSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    //initalializes the imu
    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu1.initialize(parameters);
        imu2.initialize(parameters);

    }

    //originally this saved the imu rile, but now I don't know its purpose, sice we can always reinit
    public void calibrateIMU() {
        BNO055IMU.CalibrationData calibrationData1 = imu1.readCalibrationData();
        File file1 = AppUtil.getInstance().getSettingsFile("AdafruitIMUCalibration1.json");
        ReadWriteFile.writeFile(file1, calibrationData1.serialize());

        BNO055IMU.CalibrationData calibrationData2 = imu2.readCalibrationData();
        File file2 = AppUtil.getInstance().getSettingsFile("AdafruitIMUCalibration2.json");
        ReadWriteFile.writeFile(file2, calibrationData2.serialize());
    }

    //uses math to the the angle from both of the imus
    public double[] getBothIMUAngle() {
        return new double[]{
                (imu1.getAngularOrientation().secondAngle + imu2.getAngularOrientation().secondAngle) / 2,
                (imu1.getAngularOrientation().thirdAngle + imu2.getAngularOrientation().thirdAngle) / 2,
                (imu1.getAngularOrientation().firstAngle + imu2.getAngularOrientation().firstAngle) / 2,
        };
    }

    //gets the angle from the one imu
    public double[] getRev2IMUAngle() {
        return new double[]{
                (imu1.getAngularOrientation().secondAngle),
                (imu1.getAngularOrientation().thirdAngle),
                (imu1.getAngularOrientation().firstAngle),
        };
    }

    //gets the angle from the
    public double[] getRev10IMUAngle() {
        return new double[]{
                (imu2.getAngularOrientation().secondAngle),
                (imu2.getAngularOrientation().thirdAngle),
                (imu2.getAngularOrientation().firstAngle),
        };
    }

    //returns the value of the touch sensor
    public boolean blockInIntake() {
        return !blockIntakeTouchSensor.getState();
    }


}
