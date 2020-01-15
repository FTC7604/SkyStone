package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
markerLatchServo -> "ml"


blockIntakeTouchSensor -> "bt"
openIntakeTouchSensor -> "it"
foundationTouchSensor -> "ft"

blikin -> "bk"

leftWingCS -> "cd"
leftCS -> "lcs"
rightCS -> "rcs"
 */

public class Robot {
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
    Servo markerLatchServo;
    private BNO055IMU imu1 = null, imu2 = null;

    private ColorSensor leftWingCS;
    private DistanceSensor leftWingDS;
    private ColorSensor leftCS;
    private DistanceSensor leftDS;
    private ColorSensor rightCS;
    private DistanceSensor rightDS;
    private COLOR_SENSOR activatedSensor;

    private DigitalChannel blockIntakeTouchSensor;
    private DigitalChannel openIntakeTouchSensor;
    private DigitalChannel foundationTouchSensor;

    private HardwareMap hardwareMap;
    private int BLUE_LINE_VALUE;
    private int RED_LINE_VALUE;


    public Robot(OpMode opMode, COLOR_SENSOR activatedSensor) {
        this.hardwareMap = opMode.hardwareMap;
        this.activatedSensor = activatedSensor;
        mapHardware();
    }

    public Robot() {
    }

    private void mapHardware() {
        //DRIVE
        rightFrontDriveMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "lf");
        leftFrontDriveMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rf");
        rightBackDriveMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "lb");
        leftBackDriveMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rb");
        rightFrontDriveMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftFrontDriveMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDriveMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDriveMotor.setDirection(DcMotorEx.Direction.FORWARD);

        //INTAKE
        rightIntakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "ri");
        leftIntakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "li");
        rightIntakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftIntakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        //ARM + LIFT
        armMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "ax");
        liftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "lx");
        liftMotor.setDirection(DcMotorEx.Direction.FORWARD);

        //PLATFORM LATCH
        leftLatchServo = hardwareMap.get(Servo.class, "ll");
        rightLatchServo = hardwareMap.get(Servo.class, "rl");
        leftLatchServo.setDirection(Servo.Direction.FORWARD);
        rightLatchServo.setDirection(Servo.Direction.REVERSE);

        //CLAW + MARKER LATCH
        blockGrabberServo = hardwareMap.get(Servo.class, "bg");
        markerLatchServo = hardwareMap.get(Servo.class, "ml");

        //IMU
        imu1 = hardwareMap.get(BNO055IMU.class, "imu");
        imu2 = hardwareMap.get(BNO055IMU.class, "imu 1");

        //INTAKE TOUCH SENSORS
        blockIntakeTouchSensor = hardwareMap.get(DigitalChannel.class, "bt");
        blockIntakeTouchSensor.setMode(DigitalChannel.Mode.INPUT);
        openIntakeTouchSensor = hardwareMap.get(DigitalChannel.class, "it");
        openIntakeTouchSensor.setMode(DigitalChannel.Mode.INPUT);
        foundationTouchSensor = hardwareMap.get(DigitalChannel.class, "ft");
        foundationTouchSensor.setMode(DigitalChannel.Mode.INPUT);

        //COLOR DISTANCE SENSOR
        switch(activatedSensor) {
            case UNDER:
                leftWingCS = hardwareMap.get(ColorSensor.class, "cd");
                leftWingDS = hardwareMap.get(DistanceSensor.class, "cd");
                break;
            case LEFT:
                leftCS = hardwareMap.get(ColorSensor.class, "lcd");
                leftDS = hardwareMap.get(DistanceSensor.class, "lcd");
                break;
            case RIGHT:
                rightCS = hardwareMap.get(ColorSensor.class, "rcd");
                rightDS = hardwareMap.get(DistanceSensor.class, "rcd");
                break;
            case NONE:
                break;
        }
    }

    //1. 560
    //2. 450
    //3. 210

    /**  ZERO POWER + RUNMODE METHODS  */
    public void setAllMotorZeroPowerProperty(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
        setDriveTrainZeroPowerProperty(zeroPowerBehavior);
        setIntakeZeroPowerProperty(zeroPowerBehavior);
        setArmZeroPowerProperty(zeroPowerBehavior);
        setLiftZeroPowerProperty(zeroPowerBehavior);
    }

    public void setAllMotorRunMode(DcMotor.RunMode runMode){
        setDriveTrainRunMode(runMode);
        setIntakeRunMode(runMode);
        setArmRunMode(runMode);
        setLiftRunMode(runMode);
    }

    public void setDriveTrainZeroPowerProperty(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        //sets the zero power behavior for the whole drive train, BRAKE, FLOAT, etc.
        leftFrontDriveMotor.setZeroPowerBehavior(zeroPowerBehavior);
        rightFrontDriveMotor.setZeroPowerBehavior(zeroPowerBehavior);
        leftBackDriveMotor.setZeroPowerBehavior(zeroPowerBehavior);
        rightBackDriveMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setDriveTrainRunMode(DcMotor.RunMode runMode) {
        //sets the run mode for the drive train, RUN_WITH_ENCODERS, STOP_AND_RESET_ENCODERS, etc.
        leftFrontDriveMotor.setMode(runMode);
        rightFrontDriveMotor.setMode(runMode);
        leftBackDriveMotor.setMode(runMode);
        rightBackDriveMotor.setMode(runMode);
    }

    public void setIntakeZeroPowerProperty(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        rightIntakeMotor.setZeroPowerBehavior(zeroPowerBehavior);
        leftIntakeMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setIntakeRunMode(DcMotor.RunMode runMode) {
        rightIntakeMotor.setMode(runMode);
        leftIntakeMotor.setMode(runMode);
    }

    public void setArmZeroPowerProperty(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        armMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setArmRunMode(DcMotor.RunMode runMode) {
        armMotor.setMode(runMode);
    }

    public void setLiftZeroPowerProperty(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        liftMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setLiftRunMode(DcMotor.RunMode runMode) {
        liftMotor.setMode(runMode);
    }

    /**  IMU METHODS  */
    //initializes IMU
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

    boolean IMUSAreCalibrated(){
        return imu1.isGyroCalibrated() && imu2.isGyroCalibrated();
    }

    //gets the average angle from both IMUs
    public double[] getBothIMUAngle() {
        return new double[]{
                (imu1.getAngularOrientation().secondAngle + imu2.getAngularOrientation().secondAngle) / 2,
                (imu1.getAngularOrientation().thirdAngle + imu2.getAngularOrientation().thirdAngle) / 2,
                (imu1.getAngularOrientation().firstAngle + imu2.getAngularOrientation().firstAngle) / 2,
        };
    }

    //gets the angle from the one IMU
    double[] getRev2IMUAngle() {
        return new double[]{
                (imu1.getAngularOrientation().secondAngle),
                (imu1.getAngularOrientation().thirdAngle),
                (imu1.getAngularOrientation().firstAngle),
        };
    }

    //gets the angle from the other IMU
    public double[] getRev10IMUAngle() {
        return new double[]{
                (imu2.getAngularOrientation().secondAngle),
                (imu2.getAngularOrientation().thirdAngle),
                (imu2.getAngularOrientation().firstAngle),
        };
    }

    public double[] getRev10IMUAngularVelocity(){
        return new double[]{
                (imu2.getAngularVelocity().xRotationRate),
                (imu2.getAngularVelocity().yRotationRate),
                (imu2.getAngularVelocity().zRotationRate),
        };
    }

    /**  SENSOR METHODS  */
    //returns the value of the touch sensor
    public boolean getBlockSensorPressed() {
        return !blockIntakeTouchSensor.getState();
    }

    public boolean getIntakeSensorPressed() {
        return !openIntakeTouchSensor.getState();
    }

    public boolean getFoundationSensorPressed() {
        return !foundationTouchSensor.getState();
    }

    public double[] getColors(){
        double[] colors = new double[4];

        switch(activatedSensor){
            case LEFT:
                double[] tempC = {leftCS.red(), leftCS.green(), leftCS.blue(), leftCS.alpha()};
                colors = tempC;
                break;
            case RIGHT:
                double[] tempD = {rightCS.red(), rightCS.green(), rightCS.blue(), rightCS.alpha()};
                colors = tempD;
                break;
            case UNDER:
                double[] tempE = {leftWingCS.red(), leftWingCS.green(), leftWingCS.blue(), leftWingCS.alpha()};
                colors = tempE;
                break;
            case NONE:
                break;
        }

        return colors;
    }

    public double getDistance(){
        double distance = 0;

        switch(activatedSensor){
            case LEFT:
                distance = leftDS.getDistance(DistanceUnit.MM);
                break;
            case RIGHT:
                distance = rightDS.getDistance(DistanceUnit.MM);
                break;
            case UNDER:
                distance = leftWingDS.getDistance(DistanceUnit.MM);
                break;
            case NONE:
                break;
        }

        return distance;
    }

    COLOR_UNDER_SENSOR color_under_sensor(){
        if(leftWingCS.blue() > BLUE_LINE_VALUE) return COLOR_UNDER_SENSOR.BLUE;
        else if(leftWingCS.red() > RED_LINE_VALUE) return COLOR_UNDER_SENSOR.RED;
        else return COLOR_UNDER_SENSOR.GRAY;
    }

}
