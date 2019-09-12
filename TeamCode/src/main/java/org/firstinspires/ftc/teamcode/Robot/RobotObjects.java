package org.firstinspires.ftc.teamcode.Robot;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

import static java.lang.Math.*;

public class RobotObjects {
    private BNO055IMU imu1 = null, imu2 = null;
    private DcMotorEx rightFrontDriveMotor, leftFrontDriveMotor, rightBackDriveMotor, leftBackDriveMotor;

    private ColorSensor rightColorSensor, leftColorSensor;
    private DistanceSensor rightDistanceSensor, leftDistanceSensor;
    private TouchSensor touchSensor;

    private View relativeLayout;

    private HardwareMap hardwareMap;

    public RobotObjects (OpMode opMode) {

        this.hardwareMap = opMode.hardwareMap;

        mapHardware();
        initIMU();
    }

    private void mapHardware() {
        driveTrainHardwareMap();
        imuHardwareMap();
        //phoneHardwareMap();
    }

    private void driveTrainHardwareMap(){
        //drivetrain
        rightFrontDriveMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "lf");
        leftFrontDriveMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rf");
        rightBackDriveMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "lb");
        leftBackDriveMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rb");

        //sets the direction for the drivetrain
        rightFrontDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackDriveMotor.setDirection(DcMotor.Direction.FORWARD);

        //so that it floats by at 0, could also be stop
        rightFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    private void imuHardwareMap(){
        imu1 = hardwareMap.get(BNO055IMU.class, "imu");
        imu2 = hardwareMap.get(BNO055IMU.class, "imu 1");
    }
    private void colorSensorHardwareMap(){
        //gets the right sensor
        rightColorSensor = hardwareMap.get(ColorSensor.class, "rcs");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rds");

        //gets the left sensor
        leftColorSensor = hardwareMap.get(ColorSensor.class, "lcs");
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "lds");
    }
    private void touchSensorHardwareMap(){
        touchSensor = hardwareMap.touchSensor.get("touch");
    }

    private void phoneHardwareMap(){
        // get a reference to the RelativeLayout so we can change the background color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
    }

    void setBackgroundColor(final int color){
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(color);
            }
        });
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
    private void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu1.initialize(parameters);
        imu2.initialize(parameters);

        calibrateIMU();

    }
    public void calibrateIMU(){
        BNO055IMU.CalibrationData calibrationData1 = imu1.readCalibrationData();
        File file1 = AppUtil.getInstance().getSettingsFile("AdafruitIMUCalibration1.json");
        ReadWriteFile.writeFile(file1, calibrationData1.serialize());

        BNO055IMU.CalibrationData calibrationData2 = imu2.readCalibrationData();
        File file2 = AppUtil.getInstance().getSettingsFile("AdafruitIMUCalibration2.json");
        ReadWriteFile.writeFile(file2, calibrationData2.serialize());
    }

    public double[] getIMUAngle(){
        return new double[]{
                (imu1.getAngularOrientation().secondAngle + imu2.getAngularOrientation().secondAngle) / 2,
                (imu1.getAngularOrientation().thirdAngle + imu2.getAngularOrientation().thirdAngle)   / 2,
                (imu1.getAngularOrientation().firstAngle + imu2.getAngularOrientation().firstAngle)   / 2,
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

    private double magnidudeMagneticFieldStrength(MagneticFlux magneticFlux){
        double magnidudeMagneticFieldStrength = 0;

        magnidudeMagneticFieldStrength += pow(magneticFlux.x,2);
        magnidudeMagneticFieldStrength += pow(magneticFlux.y,2);
        magnidudeMagneticFieldStrength += pow(magneticFlux.z,2);

        return sqrt(magnidudeMagneticFieldStrength);
    }
    public double getMagneticFieldStrength(){
        return (magnidudeMagneticFieldStrength(imu1.getMagneticFieldStrength()) + magnidudeMagneticFieldStrength(imu2.getMagneticFieldStrength()))/2;
    }
    public double getTemprature(){
        return (imu1.getTemperature().temperature + imu2.getTemperature().temperature)/2;
    }

    /**
     * Everything that both color distance sensors could ever offer you:
     * Distance
     * ARGB, for objective look
     * RGB, for compulation
     * HSV, for human look
     */
    public double getRightDistance(){
        return rightDistanceSensor.getDistance(DistanceUnit.METER);
    }
    public double getLeftDistance(){
        return leftDistanceSensor.getDistance(DistanceUnit.METER);
    }

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

    private double[] getRightARBG(){
        return (getARGB(rightColorSensor));
    }
    private double[] getLeftARBG(){
        return (getARGB(leftColorSensor));
    }
    private double[] getRightRBG(){
        return (getRGB(rightColorSensor));
    }
    private double[] getLeftRBG(){
        return (getRGB(leftColorSensor));
    }
    private double[] getRightHSV(){
        return (getHSV(rightColorSensor));
    }
    private double[] getLeftHSV(){
        return (getHSV(leftColorSensor));
    }


    public boolean isRight(double[] RGB, int precision){
        return isColor(RGB, precision, true);
    }
    public boolean isLeft(double[] RGB, int precision){
        return isColor(RGB, precision, false);
    }

    private double colorDifference(double[] RGB1, double[] RGB2){
        double colorDifference = 0;

        double redMean = (RGB1[0] + RGB2[0])/2;
        double redDifference = RGB1[0] - RGB2[0];
        double greenDifference = RGB1[1] - RGB2[1];
        double blueDifference = RGB1[2] - RGB2[2];

        colorDifference += (2 + redMean/256) * pow(redDifference, 2);
        colorDifference += 4 * greenDifference;
        colorDifference+= (2 + ((255-redDifference)/256)) * pow(blueDifference, 2);
        colorDifference = sqrt(colorDifference);

        return colorDifference;
    }
    private boolean isColor(double[] RGB, double precision, boolean isRight){
        if(isRight){
            return colorDifference(getRightRBG(), RGB) < precision;
        }
        else {
            return colorDifference(getLeftRBG(), RGB) < precision;
        }
    }

    private boolean isPressed(){
        return touchSensor.isPressed();
    }

    //sets the powers on either a power or velocity, and with variables or an array, where 0 is x, 1 is y, and 2 is r
    public void mecPowerDrive(double x, double y, double r){
        leftFrontDriveMotor.setPower(y - x + r);
        leftBackDriveMotor.setPower(y + x + r);
        rightFrontDriveMotor.setPower(y + x - r);
        rightBackDriveMotor.setPower(y - x - r);
    }
    public void mecPowerDrive(double[] controller){
        leftFrontDriveMotor.setPower(controller[1] - controller[0] + controller[2]);
        leftBackDriveMotor.setPower(controller[1] + controller[0] + controller[2]);
        rightFrontDriveMotor.setPower(controller[1] + controller[0] - controller[2]);
        rightBackDriveMotor.setPower(controller[1] - controller[0] - controller[2]);
    }
    public void mecVelocityDrive(double x, double y, double r){
        leftFrontDriveMotor.setVelocity(y - x + r);
        leftBackDriveMotor.setVelocity(y + x + r);
        rightFrontDriveMotor.setVelocity(y + x - r);
        rightBackDriveMotor.setVelocity(y - x - r);
    }
    public void mecVelocityDrive(double[] controller){
        leftFrontDriveMotor.setVelocity(controller[1] - controller[0] + controller[2]);
        leftBackDriveMotor.setVelocity(controller[1] + controller[0] + controller[2]);
        rightFrontDriveMotor.setVelocity(controller[1] + controller[0] - controller[2]);
        rightBackDriveMotor.setVelocity(controller[1] - controller[0] - controller[2]);
    }
}
