package org.firstinspires.ftc.teamcode.Robot;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    private TouchSensor blockIntakeTouchSensor;

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

        //latchHardwareMap();
        //blockGrabberHardwareMap();

        imuHardwareMap();
        blockIntakeTouchSensorHardwareMap();
    }

    private void driveTrainHardwareMap(){
        //drivetrain

        rightFrontDriveMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "lf");
        leftFrontDriveMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rf");
        rightBackDriveMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "lb");
        leftBackDriveMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rb");

        //sets the direction for the drivetrain
        rightFrontDriveMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftFrontDriveMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDriveMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDriveMotor.setDirection(DcMotorEx.Direction.FORWARD);

    }
    public void setDriveTrainZeroPowerProperty(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
        leftFrontDriveMotor.setZeroPowerBehavior(zeroPowerBehavior);
        rightFrontDriveMotor.setZeroPowerBehavior(zeroPowerBehavior);
        leftBackDriveMotor.setZeroPowerBehavior(zeroPowerBehavior);
        rightBackDriveMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }
    public void setDriveTrainRunMode(DcMotor.RunMode runMode){
        leftFrontDriveMotor.setMode(runMode);
        rightFrontDriveMotor.setMode(runMode);
        leftBackDriveMotor.setMode(runMode);
        rightBackDriveMotor.setMode(runMode);
    }

    private void intakeHardwareMap(){
        //intake
        rightIntakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "ri");
        leftIntakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "li");

        //sets the direction of the intake
        rightIntakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftIntakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
    }
    public void setIntakeZeroPowerProperty(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior){
        //so that it floats by at 0, could also be stop
        rightIntakeMotor.setZeroPowerBehavior(zeroPowerBehavior);
        leftIntakeMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }
    public void setIntakeRunMode(DcMotor.RunMode runMode){
        rightIntakeMotor.setMode(runMode);
        leftIntakeMotor.setMode(runMode);
    }

    private void armHardwareMap(){
        //intake
        armMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "ax");
    }
    public void setArmZeroPowerProperty(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior){
        //so that it floats by at 0, could also be stop
        armMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }
    public void setArmRunMode(DcMotor.RunMode runMode){
        armMotor.setMode(runMode);
    }

    private void liftHardwareMap(){
        //intake
        liftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "lx");

        liftMotor.setDirection(DcMotorEx.Direction.FORWARD);
    }
    public void setLiftZeroPowerProperty(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior){
        //so that it floats by at 0, could also be stop
        liftMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }
    public void setLiftRunMode(DcMotor.RunMode runMode){
        liftMotor.setMode(runMode);
    }

    private void latchHardwareMap(){
        leftLatchServo = hardwareMap.get(Servo.class, "ll");
        rightLatchServo = hardwareMap.get(Servo.class,"rl");

        leftLatchServo.setDirection(Servo.Direction.FORWARD);
        rightLatchServo.setDirection(Servo.Direction.REVERSE);
    }

    private void blockGrabberHardwareMap(){
        blockGrabberServo = hardwareMap.get(Servo.class, "bg");
    }


    //methods that will serve us in the future, so we don't need to think about anything
    private void imuHardwareMap(){
        imu1 = hardwareMap.get(BNO055IMU.class, "imu");
        imu2 = hardwareMap.get(BNO055IMU.class, "imu 1");
    }
//    private void colorSensorHardwareMap(){
//        //gets the right sensor
//        colorSensor = hardwareMap.get(ColorSensor.class, "rcs");
//        distanceSensor = hardwareMap.get(DistanceSensor.class, "rds");
//    }
    private void blockIntakeTouchSensorHardwareMap(){
        //blockIntakeTouchSensor = hardwareMap.touchSensor.get("bt");
        blockIntakeTouchSensor = hardwareMap.get(TouchSensor.class,"bt");
    }

    /**
     * All of the IMU methods that one could ever need, 0 is x, 1 is y, 2 is z:
     * Initialization and Calibration
     * Angle
     * Angular Velocity
     * Position
     * Velocity
     * Acceleration
     * Smoothed Acceleration
     * Temprature and Magnetic Field
     */
    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu1.initialize(parameters);
        imu2.initialize(parameters);

    }
    public void calibrateIMU(){
        BNO055IMU.CalibrationData calibrationData1 = imu1.readCalibrationData();
        File file1 = AppUtil.getInstance().getSettingsFile("AdafruitIMUCalibration1.json");
        ReadWriteFile.writeFile(file1, calibrationData1.serialize());

        BNO055IMU.CalibrationData calibrationData2 = imu2.readCalibrationData();
        File file2 = AppUtil.getInstance().getSettingsFile("AdafruitIMUCalibration2.json");
        ReadWriteFile.writeFile(file2, calibrationData2.serialize());
    }

    public double[] getBothIMUAngle(){
        return new double[]{
                (imu1.getAngularOrientation().secondAngle + imu2.getAngularOrientation().secondAngle) / 2,
                (imu1.getAngularOrientation().thirdAngle + imu2.getAngularOrientation().thirdAngle)   / 2,
                (imu1.getAngularOrientation().firstAngle + imu2.getAngularOrientation().firstAngle)   / 2,
        };
    }
    public double[] getRev2IMUAngle(){
        return new double[]{
                (imu1.getAngularOrientation().secondAngle) ,
                (imu1.getAngularOrientation().thirdAngle) ,
                (imu1.getAngularOrientation().firstAngle) ,
        };
    }
    public double[] getRev10IMUAngle(){
        return new double[]{
                (imu2.getAngularOrientation().secondAngle),
                (imu2.getAngularOrientation().thirdAngle),
                (imu2.getAngularOrientation().firstAngle),
        };
    }

    public double[] getIMUAnglularVelocity(){

        return new double[]{
                (imu1.getAngularVelocity().xRotationRate + imu2.getAngularVelocity().xRotationRate) / 2,
                (imu1.getAngularVelocity().yRotationRate + imu2.getAngularVelocity().yRotationRate )/ 2,
                (imu1.getAngularVelocity().zRotationRate + imu2.getAngularVelocity().zRotationRate) / 2,
        };
    }
    public double[] getIMUPositition(){

        return new double[]{
                (imu1.getPosition().x + imu2.getPosition().x) / 2,
                (imu1.getPosition().y + imu2.getPosition().y) / 2,
                (imu1.getPosition().z + imu2.getPosition().z) / 2,
        };
    }
    public double[] getIMUVelocity(){
        return new double[]{
                (imu1.getVelocity().xVeloc + imu2.getVelocity().xVeloc) / 2,
                (imu1.getVelocity().yVeloc + imu2.getVelocity().yVeloc) / 2,
                (imu1.getVelocity().zVeloc + imu2.getVelocity().zVeloc) / 2,
        };
    }
    public double[] getIMUAcceleration(){
        return new double[]{
                (imu1.getAcceleration().xAccel + imu2.getAcceleration().xAccel)/2,
                (imu1.getAcceleration().yAccel + imu2.getAcceleration().yAccel)/2,
                (imu1.getAcceleration().zAccel + imu2.getAcceleration().zAccel)/2,
        };
    }
    public double[] getIMULinearAcceleration(){
        return new double[]{
                (imu1.getLinearAcceleration().xAccel + imu2.getLinearAcceleration().xAccel)/2,
                (imu1.getLinearAcceleration().yAccel + imu2.getLinearAcceleration().yAccel)/2,
                (imu1.getLinearAcceleration().zAccel + imu2.getLinearAcceleration().zAccel)/2,
        };
    }

    /**
     * Everything that both color distance sensors could ever offer you:
     * Distance
     * ARGB, for objective look
     * RGB, for computation
     * HSV, for human look
     */
//    public double getRightDistance(){
//        return distanceSensor.getDistance(DistanceUnit.INCH);
//    }

    private double[] getARGB(ColorSensor colorSensor){
        return new double[]{
                (double) colorSensor.alpha() * 255,
                (double) colorSensor.red()   * 255,
                (double) colorSensor.green() * 255,
                (double) colorSensor.blue()  * 255
        };
    }
    private double[] getRGB(ColorSensor colorSensor){
        return new double[]{
                (double) colorSensor.red() * 255,
                (double) colorSensor.green() * 255,
                (double) colorSensor.blue() * 255
        };
    }
    private double[] getHSV(ColorSensor colorSensor){
        float[] floatHSV = new float[3];
        double[] doubleHSV = new double[3];

        //coverts the color that it gets into HSV
        Color.RGBToHSV(
                (int)((double)colorSensor.red() * 255),
                (int)((double)colorSensor.green() * 255),
                (int)((double)colorSensor.blue() * 255),

                floatHSV
        );

        //casts the doubles into floats.
        for(int i = 0; i < 3; i++) doubleHSV[i] = (double)floatHSV[i];

        return doubleHSV;

    }
//    private double[] getARBG(){
//        return (getARGB(colorSensor));
//    }
//    private double[] getRightRBG(){
//        return (getRGB(colorSensor));
//    }
//    private double[] getHSV(){
//        return (getHSV(colorSensor));
//    }
//
    public boolean blockIntakeTouchSensorIsPressed(){
        return blockIntakeTouchSensor.isPressed();
    }


}
