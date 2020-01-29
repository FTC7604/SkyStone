package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Control.BallisticMotionProfile;
import org.firstinspires.ftc.teamcode.Control.BetterBalisticProfile;
import org.firstinspires.ftc.teamcode.IO.PropertiesLoader;

import static java.lang.Math.abs;
import static java.lang.Math.signum;
import static org.firstinspires.ftc.teamcode.Control.BetterBalisticProfile.CURVE_TYPE.LINEAR;
import static org.firstinspires.ftc.teamcode.Control.BetterBalisticProfile.CURVE_TYPE.SINUSOIDAL_SCURVE;

//changes this commit
//removed unused thread methods
//changed to use average encoder vals
//testing experimental angle compensation while moving forward/backward

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

    private double PRECISE_ROTATION_START_POWER = propertiesLoader.getDoubleProperty("PRECISE_ROTATION_START_POWER");
    private double PRECISE_ROTATION_END_POWER = propertiesLoader.getDoubleProperty("PRECISE_ROTATION_END_POWER");

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

    /**
     * SIDE GRABBER POSITIONS
     */
    private double LEFT_STOWED_GRABBER = propertiesLoader.getDoubleProperty("LEFT_STOWED_GRABBER");
    private double LEFT_STOWED_SERVO = propertiesLoader.getDoubleProperty("LEFT_STOWED_SERVO");
    private double LEFT_GRABBING_GRABBER = propertiesLoader.getDoubleProperty("LEFT_GRABBING_GRABBER");
    private double LEFT_GRABBING_SERVO = propertiesLoader.getDoubleProperty("LEFT_GRABBING_SERVO");
    private double LEFT_READY_GRABBER = propertiesLoader.getDoubleProperty("LEFT_READY_GRABBER");
    private double LEFT_READY_SERVO = propertiesLoader.getDoubleProperty("LEFT_READY_SERVO");
    private double LEFT_DEFAULT_GRABBER = propertiesLoader.getDoubleProperty("LEFT_DEFAULT_GRABBER");
    private double LEFT_DEFAULT_SERVO = propertiesLoader.getDoubleProperty("LEFT_DEFAULT_SERVO");
    private double RIGHT_STOWED_GRABBER = propertiesLoader.getDoubleProperty("RIGHT_STOWED_GRABBER");
    private double RIGHT_STOWED_SERVO = propertiesLoader.getDoubleProperty("RIGHT_STOWED_SERVO");
    private double RIGHT_GRABBING_GRABBER = propertiesLoader.getDoubleProperty("RIGHT_GRABBING_GRABBER");
    private double RIGHT_GRABBING_SERVO = propertiesLoader.getDoubleProperty("RIGHT_GRABBING_SERVO");
    private double RIGHT_READY_GRABBER = propertiesLoader.getDoubleProperty("RIGHT_READY_GRABBER");
    private double RIGHT_READY_SERVO = propertiesLoader.getDoubleProperty("RIGHT_READY_SERVO");
    private double RIGHT_DEFAULT_GRABBER = propertiesLoader.getDoubleProperty("RIGHT_DEFAULT_GRABBER");
    private double RIGHT_DEFAULT_SERVO = propertiesLoader.getDoubleProperty("RIGHT_DEFAULT_SERVO");

    private BetterBalisticProfile preciseRrotationBetterBalisticProfile = new BetterBalisticProfile(ROTATION_ACCELERATION_DISTANCE, ROTATION_DECELLERATION_DISTANCE, PRECISE_ROTATION_START_POWER, ROTATION_FULL_POWER, PRECISE_ROTATION_END_POWER, SINUSOIDAL_SCURVE, LINEAR);
    private BetterBalisticProfile fastRotationBetterBalisticProfile = new BetterBalisticProfile(ROTATION_ACCELERATION_DISTANCE, ROTATION_DECELLERATION_DISTANCE, ROTATION_START_POWER, ROTATION_FULL_POWER, ROTATION_END_POWER, SINUSOIDAL_SCURVE, LINEAR);
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

    public void compensatedMecanumPowerDrive(double strafe, double forward, double rotation, double ratio){

        if(ratio < 1) {
            leftFrontDriveMotor.setPower((forward - strafe + rotation) * ratio);
            leftBackDriveMotor.setPower((forward + strafe + rotation) * ratio);
            rightFrontDriveMotor.setPower(forward + strafe - rotation);
            rightBackDriveMotor.setPower(forward - strafe - rotation);
        } else{
            leftFrontDriveMotor.setPower((forward - strafe + rotation));
            leftBackDriveMotor.setPower((forward + strafe + rotation));
            rightFrontDriveMotor.setPower((forward + strafe - rotation) / ratio);
            rightBackDriveMotor.setPower((forward - strafe - rotation) / ratio);
        }

    }

    public void stopDriveMotors(){
        mecanumPowerDrive(0, 0, 0);
    }


    public void turnToDegreePrecise(double desiredEndRotation){

        double startRotation = getRev10IMUAngle()[2];
        double currentRotation;
        double motorPower;

        if(abs(desiredEndRotation - startRotation + 360) < abs(desiredEndRotation - startRotation)){
            desiredEndRotation += 360;
        } else if(abs(desiredEndRotation - startRotation - 360) < abs(desiredEndRotation - startRotation)){
            desiredEndRotation -= 360;
        }

        preciseRrotationBetterBalisticProfile.setCurve(startRotation, desiredEndRotation,PRECISE_ROTATION_START_POWER,PRECISE_ROTATION_END_POWER);

        while(!preciseRrotationBetterBalisticProfile.isDone() && linearOpMode.opModeIsActive()){
            currentRotation = getRev10IMUAngle()[2];

            if(desiredEndRotation < 0 && currentRotation > 0){
                currentRotation -= 360;
            } else if(desiredEndRotation > 0 && currentRotation < 0){
                currentRotation += 360;
            }

            preciseRrotationBetterBalisticProfile.setCurrentPosition(currentRotation);

            motorPower = preciseRrotationBetterBalisticProfile.getCurrentPowerAccelDecel();
            mecanumPowerDrive(MOVEMENT_DIRECTION.ROTATION, motorPower);

            //linearOpMode.telemetry.addLine("Motor Power:" + motorPower);
            //linearOpMode.telemetry.update();

            linearOpMode.telemetry.addData("Rotation", currentRotation);
            linearOpMode.telemetry.addData("Power", motorPower);
            linearOpMode.telemetry.update();
        }

        preciseRrotationBetterBalisticProfile.getRidOfTemp();
        stopDriveMotors();
    }

    public void turnToDegreeFast(double desiredEndRotation){

        double startRotation = getRev10IMUAngle()[2];
        double currentRotation;
        double motorPower;

        if(abs(desiredEndRotation - startRotation + 360) < abs(desiredEndRotation - startRotation)){
            desiredEndRotation += 360;
        } else if(abs(desiredEndRotation - startRotation - 360) < abs(desiredEndRotation - startRotation)){
            desiredEndRotation -= 360;
        }

        fastRotationBetterBalisticProfile.setCurve(startRotation, desiredEndRotation);

        while(!fastRotationBetterBalisticProfile.isDone() && linearOpMode.opModeIsActive()){
            currentRotation = getRev10IMUAngle()[2];

            if(desiredEndRotation < 0 && currentRotation > 0){
                currentRotation -= 360;
            } else if(desiredEndRotation > 0 && currentRotation < 0){
                currentRotation += 360;
            }

            fastRotationBetterBalisticProfile.setCurrentPosition(currentRotation);

            motorPower = fastRotationBetterBalisticProfile.getCurrentPowerAccelDecel();
            mecanumPowerDrive(MOVEMENT_DIRECTION.ROTATION, motorPower);

            //linearOpMode.telemetry.addLine("Motor Power:" + motorPower);
            //linearOpMode.telemetry.update();

            linearOpMode.telemetry.addData("Rotation", currentRotation);
            linearOpMode.telemetry.addData("Power", motorPower);
            linearOpMode.telemetry.update();
        }

        if(fastRotationBetterBalisticProfile.getEnd_power() == 0) {
            stopDriveMotors();
        }

        fastRotationBetterBalisticProfile.getRidOfTemp();
    }

    private double calcRatio(double desiredAngle){
        double currentAngle = getRev10IMUAngle()[2];
        double testDif = currentAngle - desiredAngle;
        double ratio;

        if(abs(currentAngle + 360 - desiredAngle) < abs(testDif)){
            testDif = currentAngle + 360 - desiredAngle;
        }

        if(abs(currentAngle - 360 - desiredAngle) < abs(testDif)){
            testDif = currentAngle - 360 - desiredAngle;
        }

        if(testDif < -90){
            ratio = 1.9;
        } else if(testDif > 90){
            ratio = 0.1;
        } else{
            ratio = (testDif / -100) + 1;
        }

        linearOpMode.telemetry.addData("Ratio", ratio);
        linearOpMode.telemetry.addData("Difference", testDif);
        linearOpMode.telemetry.update();
        return ratio;
    }

    /**
     * Steers robot ever so slightly during movement to account for rotation differences
     */
    public void compensatingMoveByInchesFast(double desiredPositionChangeInInches, MOVEMENT_DIRECTION movement_direction, double desiredAngle){
        double startPosition = getAverageDriveTrainEncoder(movement_direction);
        double endPosition   = startPosition + desiredPositionChangeInInches * inchesToEncoders;
        double currentPosition;
        double motorPower;

        BetterBalisticProfile betterBalisticProfile;

        switch (movement_direction) {
            case STRAFE:
                betterBalisticProfile = strafeBetterBalisticProfile;
                break;
            default:
                betterBalisticProfile = forwardBetterBalisticProfile;
                break;
        }

        betterBalisticProfile.setCurve(startPosition, endPosition);

        while(!betterBalisticProfile.isDone() && linearOpMode.opModeIsActive()){
            currentPosition = getAverageDriveTrainEncoder(movement_direction);
            betterBalisticProfile.setCurrentPosition(currentPosition);
            motorPower = betterBalisticProfile.getCurrentPowerAccelDecel();

            if(movement_direction != MOVEMENT_DIRECTION.FORWARD) {
                mecanumPowerDrive(movement_direction, motorPower);
            } else{
                compensatedMecanumPowerDrive(0, motorPower, 0, calcRatio(desiredAngle));
            }

        }

        if(betterBalisticProfile.getEnd_power() == 0) {
            stopDriveMotors();
        }

        betterBalisticProfile.getRidOfTemp();
    }

    public void moveByInchesFast(double desiredPositionChangeInInches, MOVEMENT_DIRECTION movement_direction){
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

        while(!betterBalisticProfile.isDone() && linearOpMode.opModeIsActive()){

            currentPosition = getAverageDriveTrainEncoder(movement_direction);
            betterBalisticProfile.setCurrentPosition(currentPosition);

            motorPower = betterBalisticProfile.getCurrentPowerAccelDecel();
            mecanumPowerDrive(movement_direction, motorPower);

            linearOpMode.telemetry.addData("Encoder", currentPosition);
            linearOpMode.telemetry.update();
        }

        if(betterBalisticProfile.getEnd_power() == 0) {
            stopDriveMotors();
        }

        betterBalisticProfile.getRidOfTemp();
    }

    @Deprecated
    public void moveByInches(double desiredPositionChangeInInches, MOVEMENT_DIRECTION movement_direction){
        moveByInches(desiredPositionChangeInInches, movement_direction, .05, 0.8);
    }

    @Deprecated
    public void moveByInchesMinPower(double desiredPositionChangeInInches, MOVEMENT_DIRECTION movement_direction, double minPower){
        moveByInches(desiredPositionChangeInInches, movement_direction, minPower, .8);

    }

    @Deprecated
    public void moveByInchesMaxPower(double desiredPositionChangeInInches, MOVEMENT_DIRECTION movement_direction, double maxPower){
        moveByInches(desiredPositionChangeInInches, movement_direction, .05, maxPower);
    }

    @Deprecated
    public void moveByInches(double desiredPositionChangeInInches, MOVEMENT_DIRECTION movement_direction, double minPower, double maxPower){
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

        } while((abs(desiredPositionChangeInEncoders - currentAverageEncoderValue) > 20) && linearOpMode.opModeIsActive());


        stopDriveMotors();
    }

    /**
     * Turns relative to starting position
     * i.e. starts at 0, 90 will always mean the same position, moving counter-clockwise
     */
    @Deprecated
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
        } else {
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

        } while((abs(desiredPositionChangeInEncoders) > abs(startEncoderValue - currentEncoderValue)) && linearOpMode.opModeIsActive());

        setArmPower(0);
    }

    /**
     * SERVO METHODS
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
        markerLatchServo.setPosition(0.3);
    }

    //LATCH IS LEFT/RIGHT PRETENDING LATCH IS FRONT OF ROBOT
    public void setLatchPosition(double pos){
        leftLatchServo.setPosition(pos);
        rightLatchServo.setPosition(pos + .05);
    }

    public void setLeftGrabberPosition(double grabberPos, double servoPos){
        leftSideGrabber.setPosition(grabberPos);
        leftSideGrabberServo.setPosition(servoPos);
    }

    public void setLeftGrabberPosition(GRABBER_POSITION pos){

        switch(pos){
            case DEFAULT:
                leftSideGrabber.setPosition(LEFT_DEFAULT_GRABBER);
                leftSideGrabberServo.setPosition(LEFT_DEFAULT_SERVO);
                break;
            case READY:
                leftSideGrabber.setPosition(LEFT_READY_GRABBER);
                leftSideGrabberServo.setPosition(LEFT_READY_SERVO);
                break;
            case GRABBING:
                leftSideGrabber.setPosition(LEFT_GRABBING_GRABBER);
                leftSideGrabberServo.setPosition(LEFT_GRABBING_SERVO);
                break;
            case STOWED:
                leftSideGrabber.setPosition(LEFT_STOWED_GRABBER);
                leftSideGrabberServo.setPosition(LEFT_STOWED_SERVO);
                break;
        }

    }

    public void setRightGrabberPosition(GRABBER_POSITION pos){

        switch(pos){
            case DEFAULT:
                rightSideGrabber.setPosition(RIGHT_DEFAULT_GRABBER);
                rightSideGrabberServo.setPosition(RIGHT_DEFAULT_SERVO);
                break;
            case READY:
                rightSideGrabber.setPosition(RIGHT_READY_GRABBER);
                rightSideGrabberServo.setPosition(RIGHT_READY_SERVO);
                break;
            case GRABBING:
                rightSideGrabber.setPosition(RIGHT_GRABBING_GRABBER);
                rightSideGrabberServo.setPosition(RIGHT_GRABBING_SERVO);
                break;
            case STOWED:
                rightSideGrabber.setPosition(RIGHT_STOWED_GRABBER);
                rightSideGrabberServo.setPosition(RIGHT_STOWED_SERVO);
                break;
        }

    }

    public void setRightGrabberPosition(double grabberPos, double servoPos){
        rightSideGrabber.setPosition(grabberPos);
        rightSideGrabberServo.setPosition(servoPos);
    }

    /**
     * ENCODER MEASUREMENT METHODS
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
                //(leftBackDriveMotor.getCurrentPosition()),
                //(leftBackDriveMotor.getCurrentPosition()),
                //(leftBackDriveMotor.getCurrentPosition()),

                (+ leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() + rightFrontDriveMotor.getCurrentPosition() + rightBackDriveMotor.getCurrentPosition()) / 4,
                (- leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() + rightFrontDriveMotor.getCurrentPosition() - rightBackDriveMotor.getCurrentPosition()) / 4,
                (+ leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() - rightFrontDriveMotor.getCurrentPosition() - rightBackDriveMotor.getCurrentPosition()) / 4,

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

    public enum GRABBER_POSITION {
        DEFAULT,
        READY,
        GRABBING,
        STOWED
    }

}