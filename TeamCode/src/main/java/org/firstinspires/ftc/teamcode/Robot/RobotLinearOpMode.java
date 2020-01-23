package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Control.BallisticMotionProfile;
import org.firstinspires.ftc.teamcode.Control.BetterBalisticProfile;
import org.firstinspires.ftc.teamcode.IO.PropertiesLoader;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.Control.BetterBalisticProfile.CURVE_TYPE.LINEAR;
import static org.firstinspires.ftc.teamcode.Control.BetterBalisticProfile.CURVE_TYPE.SINUSOIDAL_NORMAL;

public class RobotLinearOpMode extends Robot {

    private PropertiesLoader propertiesLoader = new PropertiesLoader("Robot");
    private LinearOpMode linearOpMode;

    public double getInchesToEncoders(){
        return inchesToEncoders;
    }

    private double inchesToEncoders = 4000 / 69; //about 60 encoder ticks to an inch

    private double encodersToInches = 69 / 4000; //about 60 encoder ticks to an inch
    private double ROTATION_ACCELERATION_DISTANCE = propertiesLoader.getDoubleProperty("ROTATION_ACCELERATION_DISTANCE");
    private double ROTATION_START_POWER = propertiesLoader.getDoubleProperty("ROTATION_START_POWER");
    private double ROTATION_DECELLERATION_DISTANCE = propertiesLoader.getDoubleProperty("ROTATION_DECELLERATION_DISTANCE");
    private double ROTATION_END_POWER = propertiesLoader.getDoubleProperty("ROTATION_END_POWER");
    private double ROTATION_FULL_POWER = propertiesLoader.getDoubleProperty("ROTATION_FULL_POWER");
    private double FORWARD_ACCELERATION_DISTANCE = propertiesLoader.getDoubleProperty("FORWARD_ACCELERATION_DISTANCE");
    private double FORWARD_START_POWER = propertiesLoader.getDoubleProperty("FORWARD_START_POWER");
    private double FORWARD_DECELLERATION_DISTANCE = propertiesLoader.getDoubleProperty("FORWARD_DECELLERATION_DISTANCE");
    private double FORWARD_END_POWER = propertiesLoader.getDoubleProperty("FORWARD_END_POWER");
    private double FORWARD_FULL_POWER = propertiesLoader.getDoubleProperty("FORWARD_FULL_POWER");
    private double STRAFE_ACCELERATION_DISTANCE = propertiesLoader.getDoubleProperty("STRAFE_ACCELERATION_DISTANCE");
    private double STRAFE_START_POWER = propertiesLoader.getDoubleProperty("STRAFE_START_POWER");
    private double STRAFE_DECELLERATION_DISTANCE = propertiesLoader.getDoubleProperty("STRAFE_DECELLERATION_DISTANCE");
    private double STRAFE_END_POWER = propertiesLoader.getDoubleProperty("STRAFE_END_POWER");
    private double STRAFE_FULL_POWER = propertiesLoader.getDoubleProperty("STRAFE_FULL_POWER");

    private double ARM_LIMIT_ONE = propertiesLoader.getDoubleProperty("ARM_LIMIT_ONE");
    private double ARM_LIMIT_TWO = propertiesLoader.getDoubleProperty("ARM_LIMIT_TWO");
    private double ARM_ACCELERATION_DISTANCE = propertiesLoader.getDoubleProperty("ARM_ACCELERATION_DISTANCE");
    private double ARM_START_POWER = propertiesLoader.getDoubleProperty("ARM_START_POWER");
    private double ARM_DECELLERATION_DISTANCE = propertiesLoader.getDoubleProperty("ARM_DECELLERATION_DISTANCE");
    private double ARM_END_POWER = propertiesLoader.getDoubleProperty("ARM_END_POWER");
    private double ARM_FULL_POWER = propertiesLoader.getDoubleProperty("ARM_FULL_POWER");

    private double LIFT_LIMIT_ONE = propertiesLoader.getDoubleProperty("LIFT_LIMIT_ONE");
    private double LIFT_LIMIT_TWO = propertiesLoader.getDoubleProperty("LIFT_LIMIT_TWO");
    private double LIFT_ACCELERATION_DISTANCE = propertiesLoader.getDoubleProperty("LIFT_ACCELERATION_DISTANCE");
    private double LIFT_START_POWER = propertiesLoader.getDoubleProperty("LIFT_START_POWER");
    private double LIFT_DECELLERATION_DISTANCE = propertiesLoader.getDoubleProperty("LIFT_DECELLERATION_DISTANCE");
    private double LIFT_END_POWER = propertiesLoader.getDoubleProperty("LIFT_END_POWER");
    private double LIFT_FULL_POWER = propertiesLoader.getDoubleProperty("LIFT_FULL_POWER");


    private BetterBalisticProfile rotationBetterBalisticProfile = new BetterBalisticProfile(ROTATION_ACCELERATION_DISTANCE, ROTATION_DECELLERATION_DISTANCE, ROTATION_START_POWER, ROTATION_FULL_POWER, ROTATION_END_POWER, SINUSOIDAL_NORMAL, SINUSOIDAL_NORMAL);
    private BetterBalisticProfile forwardBetterBalisticProfile = new BetterBalisticProfile(FORWARD_ACCELERATION_DISTANCE, FORWARD_DECELLERATION_DISTANCE, FORWARD_START_POWER, FORWARD_FULL_POWER, FORWARD_END_POWER, LINEAR, LINEAR);
    private BetterBalisticProfile strafeBetterBalisticProfile = new BetterBalisticProfile(STRAFE_ACCELERATION_DISTANCE, STRAFE_DECELLERATION_DISTANCE, STRAFE_START_POWER, STRAFE_FULL_POWER, STRAFE_END_POWER, LINEAR, LINEAR);
    private BetterBalisticProfile armBetterBalisticProfile = new BetterBalisticProfile(ARM_ACCELERATION_DISTANCE, ARM_DECELLERATION_DISTANCE, ARM_START_POWER, ARM_FULL_POWER, ARM_END_POWER, ARM_LIMIT_ONE, ARM_LIMIT_TWO, LINEAR, LINEAR);
    private BetterBalisticProfile liftBetterBalisticProfile = new BetterBalisticProfile(LIFT_ACCELERATION_DISTANCE, LIFT_DECELLERATION_DISTANCE, LIFT_START_POWER, LIFT_FULL_POWER, LIFT_END_POWER, LIFT_LIMIT_ONE, LIFT_LIMIT_TWO, LINEAR, LINEAR);


    /**
     * CONSTRUCTORS
     */


    public RobotLinearOpMode(LinearOpMode linearOpMode){

        super(linearOpMode);
        this.linearOpMode = linearOpMode;
        init();
    }

    private void init(){
        linearOpMode.telemetry.addData("Status", " DO NOT START");
        linearOpMode.telemetry.update();

        setAllMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setAllMotorZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);

        initIMU();
        calibration();

        holdMarker();
        openGrabber();

        linearOpMode.telemetry.addData("Status", "Initialized");
        linearOpMode.telemetry.update();
    }

    /**
     * DRIVE MOTOR METHODS
     */

    private void mecanumPowerDrive(MOVEMENT_DIRECTION movement_direction, double power){
        switch (movement_direction) {
            case STRAFE:
                mecanumPowerDrive(power, 0, 0);
                break;
            case FORWARD:
                mecanumPowerDrive(0, power, 0);
                break;
            case ROTATION:
                mecanumPowerDrive(0, 0, power);
                break;
        }
    }


    public void mecanumPowerDrive(double strafe, double forward, double rotation){
        leftFrontDriveMotor.setPower(forward - strafe + rotation);
        leftBackDriveMotor.setPower(forward + strafe + rotation);
        rightFrontDriveMotor.setPower(forward + strafe - rotation);
        rightBackDriveMotor.setPower(forward - strafe - rotation);
    }


    private void stopDriveMotors(){
        mecanumPowerDrive(0, 0, 0);
    }

    public void turnToDegreeFast(double desiredEndRotation){

        double startRotation = getRev10IMUAngle()[2];
        double currentRotation;
        double motorPower;

        rotationBetterBalisticProfile.setCurve(startRotation, desiredEndRotation);

        while((rotationBetterBalisticProfile.isNotDone() || getRev10IMUAngularVelocity()[2] > rotationBetterBalisticProfile.getEnd_power() * 90) && linearOpMode.opModeIsActive()){

            currentRotation = getRev10IMUAngle()[2];
            rotationBetterBalisticProfile.setCurrentPosition(currentRotation);

            motorPower = rotationBetterBalisticProfile.getCurrentPowerAccelDecel();
            mecanumPowerDrive(MOVEMENT_DIRECTION.ROTATION, motorPower);

            linearOpMode.telemetry.addLine("Motor Power:" + motorPower);
            linearOpMode.telemetry.update();
        }
    }

    public void moveByInchesFast(
            double desiredPositionChangeInInches, MOVEMENT_DIRECTION movement_direction
    ){

        double startPosition = getAverageDriveTrainEncoder(movement_direction);
        double endPosition   = startPosition + desiredPositionChangeInInches * inchesToEncoders;
        double currentPosition;
        double motorPower;

        BetterBalisticProfile betterBalisticProfile;

        switch (movement_direction) {
            case STRAFE:
                betterBalisticProfile = strafeBetterBalisticProfile;
                break;
            case FORWARD:
            default:
                betterBalisticProfile = forwardBetterBalisticProfile;
        }

        betterBalisticProfile.setCurve(startPosition, endPosition);

        while((betterBalisticProfile.isNotDone()) && linearOpMode.opModeIsActive()){

            currentPosition = getAverageDriveTrainEncoder(movement_direction);
            betterBalisticProfile.setCurrentPosition(currentPosition);

            motorPower = betterBalisticProfile.getCurrentPowerAccelDecel();
            mecanumPowerDrive(movement_direction, motorPower);

            linearOpMode.telemetry.addLine("Motor Power:" + motorPower);
            linearOpMode.telemetry.update();
        }

        if(betterBalisticProfile.getEnd_power() == 0) {
            stopDriveMotors();
        }
    }

    public void moveByInches(
            double desiredPositionChangeInInches, MOVEMENT_DIRECTION movement_direction
    ){
        moveByInches(desiredPositionChangeInInches, movement_direction, .05, 0.8);
    }

    public void moveByInchesMinPower(
            double desiredPositionChangeInInches,
            MOVEMENT_DIRECTION movement_direction,
            double minPower
    ){
        moveByInches(desiredPositionChangeInInches, movement_direction, minPower, .8);

    }

    public void moveByInchesMaxPower(
            double desiredPositionChangeInInches,
            MOVEMENT_DIRECTION movement_direction,
            double maxPower
    ){
        moveByInches(desiredPositionChangeInInches, movement_direction, .05, maxPower);
    }

    public void moveByInches(
            double desiredPositionChangeInInches,
            MOVEMENT_DIRECTION movement_direction,
            double minPower,
            double maxPower
    ){
        double currentAverageEncoderValue;
        double adjustedMotorPower;
        double startDriveTrainEncoders;

        BallisticMotionProfile DriveProfile = new BallisticMotionProfile(0, 0, 400, minPower, 1, maxPower);

        double desiredPositionChangeInEncoders = desiredPositionChangeInInches * inchesToEncoders;

        setDriveTrainRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveTrainRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        startDriveTrainEncoders = getAverageDriveTrainEncoder(movement_direction);

        do {
            currentAverageEncoderValue = getAverageDriveTrainEncoder(movement_direction);
            adjustedMotorPower         = DriveProfile.RunToPositionWithAccel(startDriveTrainEncoders, currentAverageEncoderValue, desiredPositionChangeInEncoders);
            mecanumPowerDrive(movement_direction, adjustedMotorPower);

        }
        while((abs(desiredPositionChangeInEncoders - currentAverageEncoderValue) > 20) && linearOpMode.opModeIsActive());


        stopDriveMotors();
    }

    /**
     * Turns relative to starting position
     * i.e. starts at 0, 90 will always mean the same position, moving counter-clockwise
     */

    public void turnToDegree(double endRotation){

        double currentRotation;
        double adjustedMotorPower;
        double startRotation;

        BallisticMotionProfile TurnProfile = new BallisticMotionProfile(0, 0, 90, .1, 1, 0.5);

        startRotation = getRev10IMUAngle()[2];

        do {

            currentRotation = getRev10IMUAngle()[2];

            linearOpMode.telemetry.addData("Heading: ", currentRotation);
            linearOpMode.telemetry.update();

            adjustedMotorPower = TurnProfile.RunToPositionWithoutAccel(startRotation, currentRotation, endRotation);

            mecanumPowerDrive(MOVEMENT_DIRECTION.ROTATION, adjustedMotorPower);


        }
        while(((abs(endRotation - currentRotation) > 2) || (abs(getRev10IMUAngularVelocity()[2]) < 10)) && linearOpMode.opModeIsActive());

        //stopAllMotors();
        stopDriveMotors();
    }

    /**
     * INTAKE+LIFT MOTOR METHODS
     */
    public void setIntakePower(double intakePower){
        if (getIntakeSensorNotPressed()) {
            rightIntakeMotor.setPower(intakePower);
            leftIntakeMotor.setPower(intakePower);
        }
        else {
            rightIntakeMotor.setPower(intakePower);
            leftIntakeMotor.setPower(-intakePower);
        }
    }

    public void setLiftPower(double liftPower){
        liftMotor.setPower(liftPower);
    }

    public void setArmPower(double armPower){
        armMotor.setPower(armPower);
    }

    public void moveArmByEncoder(double desiredPositionChangeInEncoders){

        double startEncoderValue = armMotor.getCurrentPosition();
        double endEncoderValue   = desiredPositionChangeInEncoders + startEncoderValue;

        double currentEncoderValue;
        double adjustedMotorPower;

        armBetterBalisticProfile.setCurve(startEncoderValue, endEncoderValue);

        do {
            currentEncoderValue = armMotor.getCurrentPosition();
            armBetterBalisticProfile.setCurrentPosition(currentEncoderValue);


            adjustedMotorPower = armBetterBalisticProfile.getCurrentPowerAccelDecel();
            setArmPower(adjustedMotorPower);

            linearOpMode.telemetry.addData("Arm Encoder", currentEncoderValue);
            linearOpMode.telemetry.update();

        }
        while((abs(desiredPositionChangeInEncoders) > abs(startEncoderValue - currentEncoderValue)) && linearOpMode.opModeIsActive());

        setArmPower(0);
    }

    public boolean isArmIsUp(){
        return armIsUp;
    }

    public volatile boolean armIsUp;
    public void threadedMoveArmByEncoder(double desiredPosition){

        Thread localThread = new Thread(() -> {
            armIsUp = false;
            moveArmByEncoder(desiredPosition);
            armIsUp = true;
        });

        localThread.start();

    }

    /**
     * LATCH+CLAW+MARKER METHODS
     */

    public void openGrabber(){
        blockGrabberServo.setPosition(.7);
    }

    public void closeGrabber(){
        blockGrabberServo.setPosition(.4);
    }

    public void dropMarker(){
        markerLatchServo.setPosition(1);
    }

    public void holdMarker(){
        markerLatchServo.setPosition(0);
    }

    //LATCH IS LEFT/RIGHT PRETENDING LATCH IS FRONT OF ROBOT
    public void setLatchPosition(double pos){
        leftLatchServo.setPosition(pos);
        rightLatchServo.setPosition(pos + .05);
    }

    /**
     * GET ENCODER METHODS
     */
    public double getAverageDriveTrainEncoder(MOVEMENT_DIRECTION movement_direction){
        switch (movement_direction) {
            case FORWARD:
                return getChangeInDriveTrainEncoder()[0];
            case STRAFE:
                return getChangeInDriveTrainEncoder()[1];
            case ROTATION:
                return getChangeInDriveTrainEncoder()[2];
        }

        return 0;
    }

    private double[] getChangeInDriveTrainEncoder(){
        return new double[] {
                (leftBackDriveMotor.getCurrentPosition()),
                (leftBackDriveMotor.getCurrentPosition()),
                (leftBackDriveMotor.getCurrentPosition()),

                //(+ leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() + rightFrontDriveMotor.getCurrentPosition() + rightBackDriveMotor.getCurrentPosition()) / 4,
                //(- leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() + rightFrontDriveMotor.getCurrentPosition() - rightBackDriveMotor.getCurrentPosition()) / 4,
                //(+ leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() - rightFrontDriveMotor.getCurrentPosition() - rightBackDriveMotor.getCurrentPosition()) / 4,

        };
    }

    //    public double[] getChangeInDriveTrainEncoder() {
    //        return new double[]{
    //        (+ leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() + rightFrontDriveMotor.getCurrentPosition() + rightBackDriveMotor.getCurrentPosition()) / 4,
    //        (- leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() + rightFrontDriveMotor.getCurrentPosition() - rightBackDriveMotor.getCurrentPosition()) / 4,
    //        (+ leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() - rightFrontDriveMotor.getCurrentPosition() - rightBackDriveMotor.getCurrentPosition()) / 4,
    //        };
    //    }

    public int getLiftEncoder(){
        return liftMotor.getCurrentPosition();
    }

    public int getArmEncoder(){
        return armMotor.getCurrentPosition();
    }

    /**
     * STOP METHODS
     */
    public void stopAllMotors(){
        leftFrontDriveMotor.setPower(0);
        leftBackDriveMotor.setPower(0);
        rightFrontDriveMotor.setPower(0);
        rightBackDriveMotor.setPower(0);
        leftIntakeMotor.setPower(0);
        rightIntakeMotor.setPower(0);
        liftMotor.setPower(0);
        armMotor.setPower(0);
    }

    private void calibration(){

        while(!IMUSAreCalibrated() && linearOpMode.opModeIsActive()){

        }

    }

    public enum MOVEMENT_DIRECTION {
        STRAFE,
        FORWARD,
        ROTATION,
    }

}