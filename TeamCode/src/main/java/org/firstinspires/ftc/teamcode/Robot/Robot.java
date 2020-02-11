package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.IO.PropertiesLoader;

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

leftSideGrabberServo -> "lsl"
leftSideGrabber -> "lsg"
rightSideGrabberServo -> "rsl"
rightSideGrabber -> "rsg"

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

    Servo leftSideGrabberServo;
    Servo leftSideGrabber;

    Servo rightSideGrabberServo;
    Servo rightSideGrabber;

    RevBlinkinLedDriver blinkin;

    private BNO055IMU imu1 = null, imu2 = null;

    private ColorSensor leftWingCS;
    private DistanceSensor leftWingDS;

    private DigitalChannel blockIntakeTouchSensor;
    private DigitalChannel openIntakeTouchSensor;
    private DigitalChannel foundationTouchSensor;

    private HardwareMap hardwareMap;

    private PropertiesLoader propertiesLoader = new PropertiesLoader("Robot");

    private double BLUE_LINE_DETECTED = propertiesLoader.getDoubleProperty("RED_LINE_DETECTED");
    private double RED_LINE_DETECTED = propertiesLoader.getDoubleProperty("BLUE_LINE_DETECTED");

    private double P = propertiesLoader.getDoubleProperty("P");
    private double I = propertiesLoader.getDoubleProperty("I");
    private double D = propertiesLoader.getDoubleProperty("D");
    private double F = propertiesLoader.getDoubleProperty("F");

    public Robot(OpMode opMode){
        this.hardwareMap = opMode.hardwareMap;
        mapHardware();
    }

    public Robot(){
    }

    void makeThread(){
        imu2.getPosition();
    }

    private void mapHardware(){
        //DRIVE
        rightFrontDriveMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "lf");
        leftFrontDriveMotor  = (DcMotorEx) hardwareMap.get(DcMotor.class, "rf");
        rightBackDriveMotor  = (DcMotorEx) hardwareMap.get(DcMotor.class, "lb");
        leftBackDriveMotor   = (DcMotorEx) hardwareMap.get(DcMotor.class, "rb");
        rightFrontDriveMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftFrontDriveMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDriveMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDriveMotor.setDirection(DcMotorEx.Direction.FORWARD);
        setDrivePIDCoefficients(P, I, D, F);

        //INTAKE
        rightIntakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "ri");
        leftIntakeMotor  = (DcMotorEx) hardwareMap.get(DcMotor.class, "li");
        rightIntakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftIntakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        //ARM + LIFT
        armMotor  = (DcMotorEx) hardwareMap.get(DcMotor.class, "ax");
        liftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "lx");
        liftMotor.setDirection(DcMotorEx.Direction.FORWARD);

        //PLATFORM LATCH
        leftLatchServo  = hardwareMap.get(Servo.class, "ll");
        rightLatchServo = hardwareMap.get(Servo.class, "rl");
        leftLatchServo.setDirection(Servo.Direction.FORWARD);
        rightLatchServo.setDirection(Servo.Direction.REVERSE);

        //CLAW + MARKER LATCH
        blockGrabberServo = hardwareMap.get(Servo.class, "bg");
        markerLatchServo  = hardwareMap.get(Servo.class, "ml");

        //SIDE GRABBERS
        leftSideGrabberServo = hardwareMap.get(Servo.class, "lsl");
        leftSideGrabber = hardwareMap.get(Servo.class, "lsg");

        rightSideGrabberServo = hardwareMap.get(Servo.class, "rsl");
        rightSideGrabber = hardwareMap.get(Servo.class, "rsg");

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

        //OTHER
        leftWingCS = hardwareMap.get(ColorSensor.class, "cd");
        leftWingDS = hardwareMap.get(DistanceSensor.class, "cd");

        try {
            blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "bk");
        } catch(Exception e){

        }

    }

    //1. 560
    //2. 450
    //3. 210

    /**
     * ZERO POWER + RUNMODE METHODS
     */
    public void setDrivePIDCoefficients(double P, double I, double D, double F){
        rightFrontDriveMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
        leftFrontDriveMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
        rightBackDriveMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
        leftBackDriveMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
    }

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


    public void setDriveTrainZeroPowerProperty(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
        //sets the zero power behavior for the whole drive train, BRAKE, FLOAT, etc.
        leftFrontDriveMotor.setZeroPowerBehavior(zeroPowerBehavior);
        rightFrontDriveMotor.setZeroPowerBehavior(zeroPowerBehavior);
        leftBackDriveMotor.setZeroPowerBehavior(zeroPowerBehavior);
        rightBackDriveMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setDriveTrainRunMode(DcMotor.RunMode runMode){
        //sets the run mode for the drive train, RUN_WITH_ENCODERS, STOP_AND_RESET_ENCODERS, etc.
        leftFrontDriveMotor.setMode(runMode);
        rightFrontDriveMotor.setMode(runMode);
        leftBackDriveMotor.setMode(runMode);
        rightBackDriveMotor.setMode(runMode);
    }

    private void setIntakeZeroPowerProperty(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior){
        rightIntakeMotor.setZeroPowerBehavior(zeroPowerBehavior);
        leftIntakeMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    private void setIntakeRunMode(DcMotor.RunMode runMode){
        rightIntakeMotor.setMode(runMode);
        leftIntakeMotor.setMode(runMode);
    }

    public void setArmZeroPowerProperty(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior){
        armMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setArmRunMode(DcMotor.RunMode runMode){
        armMotor.setMode(runMode);
    }

    public void setLiftZeroPowerProperty(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior){
        liftMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setLiftRunMode(DcMotor.RunMode runMode){
        liftMotor.setMode(runMode);
    }

    /**
     * IMU METHODS
     */
    //initializes IMU
    public void initIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        parameters.angleUnit      = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit      = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu1.initialize(parameters);
        imu2.initialize(parameters);

        parameters.mode = BNO055IMU.SensorMode.CONFIG;
    }

    //originally this saved the imu rile, but now I don't know its purpose, sice we can always reinit
    public File[] readIMUCalibrationFiles(){
        File[] calibrationFiles = new File[2];
        BNO055IMU.CalibrationData[] calibrationData = new BNO055IMU.CalibrationData[2];

        calibrationData[0] = imu1.readCalibrationData();
        calibrationData[1] = imu2.readCalibrationData();

        calibrationFiles[0] = AppUtil.getInstance().getSettingsFile("AdafruitIMUCalibration2.json");
        calibrationFiles[1] = AppUtil.getInstance().getSettingsFile("AdafruitIMUCalibration1.json");

        ReadWriteFile.writeFile(calibrationFiles[0], calibrationData[0].serialize());
        ReadWriteFile.writeFile(calibrationFiles[1], calibrationData[1].serialize());

        return calibrationFiles;
    }

    public void setCalibrationData(File[] calibrationFiles){
        String[] data = new String[2];
        BNO055IMU.CalibrationData[] calibrationData = new BNO055IMU.CalibrationData[2];

        data[0] = ReadWriteFile.readFile(calibrationFiles[0]);
        data[1] = ReadWriteFile.readFile(calibrationFiles[1]);

        calibrationData[0] = BNO055IMU.CalibrationData.deserialize(data[0]);
        calibrationData[1] = BNO055IMU.CalibrationData.deserialize(data[1]);

        imu1.writeCalibrationData(calibrationData[0]);
        imu2.writeCalibrationData(calibrationData[1]);
    }

    boolean IMUSAreCalibrated(){
        return imu1.isGyroCalibrated() && imu2.isGyroCalibrated();
    }

    //gets the average angle from both IMUs
    public double[] getBothIMUAngle(){
        return new double[] {
                (imu1.getAngularOrientation().secondAngle + imu2.getAngularOrientation().secondAngle) / 2,
                (imu1.getAngularOrientation().thirdAngle + imu2.getAngularOrientation().thirdAngle) / 2,
                (imu1.getAngularOrientation().firstAngle + imu2.getAngularOrientation().firstAngle) / 2,};
    }

    //gets the angle from the one IMU
    double[] getRev2IMUAngle(){
        return new double[] {
                (imu1.getAngularOrientation().secondAngle),
                (imu1.getAngularOrientation().thirdAngle),
                (imu1.getAngularOrientation().firstAngle),};
    }

    //gets the angle from the other IMU
    double[] getRev10IMUAngle(){
        return new double[] {
                (imu2.getAngularOrientation().secondAngle),
                (imu2.getAngularOrientation().thirdAngle),
                (imu2.getAngularOrientation().firstAngle),};
    }

    double[] getRev10IMUAngularVelocity(){
        return new double[] {
                (imu2.getAngularVelocity().xRotationRate),
                (imu2.getAngularVelocity().yRotationRate),
                (imu2.getAngularVelocity().zRotationRate),};
    }

    double getRev10IMUSpeed(){
        double answer = 0;

        answer += Math.pow(imu2.getVelocity().xVeloc, 2);
        answer += Math.pow(imu2.getVelocity().yVeloc, 2);
        answer = Math.sqrt(answer);

        //converts to inches
        answer *= 39.37;

        return answer;


    }


    /**
     * SENSOR METHODS
     */
    //returns the value of the touch sensor
    public boolean getBlockSensorNotPressed(){
        return blockIntakeTouchSensor.getState();
    }

    boolean getIntakeSensorNotPressed(){
        return openIntakeTouchSensor.getState();
    }

    public boolean getFoundationSensorPressed(){
        return !foundationTouchSensor.getState();
    }

//    public double getVoltage(){
//        hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
//    }

    public double[] getColors(){
        double[] colors = new double[4];

        double[] tempE = {
                leftWingCS.red(), leftWingCS.green(), leftWingCS.blue(), leftWingCS.alpha()};

        return colors;
    }

    public double getDistance(){

        return leftWingDS.getDistance(DistanceUnit.MM);
    }

    public COLOR_UNDER_SENSOR color_under_sensor(){
        if (leftWingCS.blue() > BLUE_LINE_DETECTED)
            return COLOR_UNDER_SENSOR.BLUE;
        else if (leftWingCS.red() > RED_LINE_DETECTED)
            return COLOR_UNDER_SENSOR.RED;
        else
            return COLOR_UNDER_SENSOR.GRAY;
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern){

        if(blinkin != null) {
            blinkin.setPattern(pattern);
        }
        
    }

}
