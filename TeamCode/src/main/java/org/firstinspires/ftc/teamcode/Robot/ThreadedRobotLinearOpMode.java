package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Control.BallisticMotionProfile;
import org.firstinspires.ftc.teamcode.IO.RuntimeLogger;

import static java.lang.Math.abs;
import static java.lang.Math.min;

public class ThreadedRobotLinearOpMode extends Robot {

    private LinearOpMode linearOpMode;

    private  double initialArmPosition = 0; //Initial encoder position of arm at resting position
    private double topArmEncoder = 2300;    //Upper limit of arm encoder
    private double bottomArmEncoder = 0;    //Lower limit of arm encoder
    private double inchesToEncoders = 4000 / 69; //about 60 encoder ticks to an inch

    private RuntimeLogger logger = new RuntimeLogger("MotionProfile");

    private double lfMotorPower = 0;
    private double lbMotorPower = 0;
    private double rfMotorPower = 0;
    private double rbMotorPower = 0;

    private Thread moveThread;
    private Thread turnThread;

    private double[] drivePowers = new double[4];
    private double[] turnPowers = new double[4];

    private Thread drivePowerThread = new Thread(() -> {
        double maxVal;

        while(true){
            lfMotorPower = drivePowers[0] + turnPowers[0];
            lbMotorPower = drivePowers[1] + turnPowers[1];
            rfMotorPower = drivePowers[2] + turnPowers[2];
            rbMotorPower = drivePowers[3] + turnPowers[3];

            if(abs(lfMotorPower) >= abs(lbMotorPower)){
                maxVal = abs(lfMotorPower);
            } else{
                maxVal = abs(lbMotorPower);
            }

            if(abs(rfMotorPower) > maxVal){
                maxVal = abs(rfMotorPower);
            }

            if(abs(rbMotorPower) > maxVal){
                maxVal = abs(rbMotorPower);
            }

            if(maxVal == 0){
                maxVal = 1;
            }

            leftFrontDriveMotor.setPower(lfMotorPower / maxVal);
            leftBackDriveMotor.setPower(lbMotorPower / maxVal);
            rightFrontDriveMotor.setPower(rfMotorPower / maxVal);
            rightBackDriveMotor.setPower(rbMotorPower / maxVal);
        }

    });

    /**  MOTION PROFILES  */
    public BallisticMotionProfile armProfile = new BallisticMotionProfile(topArmEncoder, bottomArmEncoder, 1000, 0.2, 1, .6);
    public BallisticMotionProfile turnProfile = new BallisticMotionProfile(0, 0, 90, .05, 1, .75);
    public BallisticMotionProfile liftProfile = new BallisticMotionProfile(
            20,
            1300,
            100,
            .05,
            1,
            .8
    );

    /**  CONSTRUCTORS  */

    public ThreadedRobotLinearOpMode(LinearOpMode linearOpMode){
        super(linearOpMode);
        this.linearOpMode = linearOpMode;
        drivePowerThread.start();
    }

    /**  DRIVE MOTOR METHODS  */
    /*private void mecanumPowerDrive(MOVEMENT_DIRECTION movement_direction, double power) {
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

    public void mecanumPowerDrive(double[] controller) {
        mecanumPowerDrive(controller[0], controller[1], controller[2]);
    }

    public void mecanumPowerDrive(double strafe, double forward, double rotation) {
        lfMotorPower += forward - strafe + rotation;
        lbMotorPower += forward + strafe + rotation;
        rfMotorPower += forward + strafe - rotation;
        rbMotorPower += forward - strafe - rotation;
    }*/

    /*public void turnByDegree(double degree) {
        double adjustedMotorPower;
        double startAngle = getRev2IMUAngle()[2];
        double neededAngle = startAngle + degree;
        double currentAngle = startAngle;

        if (currentAngle < neededAngle) {

            while ((currentAngle < neededAngle && linearOpMode.opModeIsActive())) {
                currentAngle = getRev2IMUAngle()[2];
                adjustedMotorPower = turnProfile.RunToPositionWithAccel(startAngle, currentAngle, neededAngle);
                mecanumPowerDrive(0, 0, adjustedMotorPower);
            }

        } else if (currentAngle > neededAngle && linearOpMode.opModeIsActive()) {

            while ((currentAngle > neededAngle)) {
                currentAngle = getRev2IMUAngle()[2];
                adjustedMotorPower = turnProfile.RunToPositionWithAccel(startAngle, currentAngle, neededAngle);
                mecanumPowerDrive(0, 0, adjustedMotorPower);
            }

        }
    }*/

    /**
     * Turns relative to starting position
     * i.e. starts at 0, 90 will always mean the same position, moving counter-clockwise
     */
    public void turnToDegree(double endRotation) {

        double currentRotation;
        double adjustedMotorPower;
        double startRotation;

        BallisticMotionProfile TurnProfile = new BallisticMotionProfile(0, 0, 90, .05, 1, 0.4);

        startRotation = getRev10IMUAngle()[2];

        if(Math.abs(endRotation - startRotation) > 180){
            startRotation += 360 * Math.signum(endRotation - startRotation);
        }

        do {
            currentRotation = getRev10IMUAngle()[2];
            adjustedMotorPower = TurnProfile.RunToPositionWithAccel(startRotation, currentRotation, endRotation);
            //adjustedMotorPower = 1;
            //mecanumPowerDrive(MOVEMENT_DIRECTION.ROTATION, adjustedMotorPower);
            turnPowers[0] = adjustedMotorPower;
            turnPowers[1] = adjustedMotorPower;
            turnPowers[2] = -adjustedMotorPower;
            turnPowers[3] = -adjustedMotorPower;
        } while ((abs(endRotation - currentRotation) > 5) && linearOpMode.opModeIsActive());

        turnPowers = new double[4];
        //stopAllMotors();
    }

    public void threadedTurnToDegree(double endRotation) {

        Thread localThread = new Thread(() -> {

            if(turnThread != null){
                while(turnThread.isAlive()){}
            }

            turnThread = new Thread(() -> {
                turnToDegree(endRotation);
            });

            turnThread.start();
        });

        localThread.start();
    }

    public void moveByInches(double desiredPositionChangeInInches, MOVEMENT_DIRECTION movement_direction) {
        moveByInches(desiredPositionChangeInInches, movement_direction, .05, 0.8);
    }

    public void moveByInches(double desiredPositionChangeInInches, MOVEMENT_DIRECTION movement_direction, double maxPower) {
        moveByInches(desiredPositionChangeInInches, movement_direction, .05, maxPower);
    }

    public void moveByInches(double desiredPositionChangeInInches, MOVEMENT_DIRECTION movement_direction, double minPower, double maxPower) {
        double currentAverageEncoderValue;
        double adjustedMotorPower;
        double startDriveTrainEncoders;
        BallisticMotionProfile DriveProfile = new BallisticMotionProfile(0, 0, 200, minPower, 1, maxPower);
        double desiredPositionChangeInEncoders = desiredPositionChangeInInches * inchesToEncoders;

        setDriveTrainRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveTrainRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        startDriveTrainEncoders = getAverageDriveTrainEncoder(movement_direction);

        do {
            currentAverageEncoderValue = getAverageDriveTrainEncoder(movement_direction);
            adjustedMotorPower = DriveProfile.RunToPositionWithAccel(startDriveTrainEncoders, currentAverageEncoderValue, desiredPositionChangeInEncoders);

            switch(movement_direction){
                case FORWARD:
                    drivePowers[0] = adjustedMotorPower;
                    drivePowers[1] = adjustedMotorPower;
                    drivePowers[2] = adjustedMotorPower;
                    drivePowers[3] = adjustedMotorPower;
                    break;
                case STRAFE:
                    drivePowers[0] = -adjustedMotorPower;
                    drivePowers[1] = adjustedMotorPower;
                    drivePowers[2] = adjustedMotorPower;
                    drivePowers[3] = -adjustedMotorPower;
                    break;
                default:
                    break;
            }

        } while ((abs(desiredPositionChangeInEncoders - currentAverageEncoderValue) > 50) && linearOpMode.opModeIsActive());

        drivePowers = new double[4];
        //stopAllMotors();
        //initIMU();
    }

    public void threadedMoveByInches(double desiredPositionChangeInInches, MOVEMENT_DIRECTION movement_direction, double minPower, double maxPower){

        Thread localThread = new Thread(() -> {

            if(moveThread != null){
                while(moveThread.isAlive()){}
            }

            moveThread = new Thread(() -> {
                moveByInches(desiredPositionChangeInInches, movement_direction, minPower, maxPower);
            });

            //initIMU();
            moveThread.start();

        });

        localThread.start();
    }

    public void waitForThreads(){

        if(moveThread != null){
            while(moveThread.isAlive()){linearOpMode.sleep(1);}
        }

        if(turnThread != null){
            while(turnThread.isAlive()){linearOpMode.sleep(1);}
        }

    }

    public enum MOVEMENT_DIRECTION {
        STRAFE,
        FORWARD,
        ROTATION,
    }

    /**  INTAKE+LIFT MOTOR METHODS  */
    public void setIntakePower(double intakePower) {
        if (getIntakeSensorNotPressed()) {
            rightIntakeMotor.setPower(intakePower);
            leftIntakeMotor.setPower(-intakePower);
        } else {
            rightIntakeMotor.setPower(intakePower);
            leftIntakeMotor.setPower(intakePower);
        }

    }

    public void setLiftPower(double liftPower) {
        liftMotor.setPower(liftPower);
    }

    public void setArmPower(double armPower) {
        armMotor.setPower(armPower);
    }

    public void moveArmByEncoder(double desiredPositionChangeInEncoders) {

        double startEncoderValue = getArmEncoder();
        double endEncoderValue = desiredPositionChangeInEncoders + startEncoderValue;

        double currentEncoderValue = 0;
        double adjustedMotorPower = 0;

        do {
            currentEncoderValue = getArmEncoder();

            adjustedMotorPower = armProfile.RunToPositionWithAccel(startEncoderValue, currentEncoderValue, endEncoderValue);

            setArmPower(adjustedMotorPower);

        } while ((abs(desiredPositionChangeInEncoders) > abs(startEncoderValue - currentEncoderValue)) && linearOpMode.opModeIsActive());
    }

    public void deploy(){
        //Deploy function
        setLiftPower(-0.2);
        linearOpMode.sleep(2000);
        setArmPower(.2);
        linearOpMode.sleep(600);
        setLiftPower(0.2);
        linearOpMode.sleep(150);
        setLiftZeroPowerProperty(DcMotor.ZeroPowerBehavior.FLOAT);
        setLiftPower(0);
        setArmPower(-0.2);
        linearOpMode.sleep(50);
        setArmZeroPowerProperty(DcMotor.ZeroPowerBehavior.FLOAT);
        setArmPower(0);
        linearOpMode.sleep(1000);
        setLiftRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setLiftRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setArmRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setArmRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Look at this method again, see if necessary
    public boolean armHasArrived(double targetEncoder) {
        boolean arrivedAtTargetEncoder;

        //if the target encoder position is below the very starting one, and the current position is below that
        //now we make a variable to use later which represents the initial position when doing a runtoposition command
        double initialArmPosition = 0;
        if ((initialArmPosition < targetEncoder) && (targetEncoder < getArmEncoder())) {
            arrivedAtTargetEncoder = true;
        }
        //the exact opposite condidion that yields essentially the same result
        else
            arrivedAtTargetEncoder = (initialArmPosition > targetEncoder) &&
                    (targetEncoder > getArmEncoder());

        return arrivedAtTargetEncoder;
    }

    /**  LATCH+CLAW+MARKER METHODS  */
    public void closeGrabber() {
        blockGrabberServo.setPosition(.7);
    }

    public void openGrabber() {
        blockGrabberServo.setPosition(.4);
    }

    public void dropMarker() {
        markerLatchServo.setPosition(1);
    }

    public void holdMarker() {
        markerLatchServo.setPosition(0);
    }

    //LATCH IS LEFT/RIGHT PRETENDING LATCH IS FRONT OF ROBOT
    public void setLatchPosition(double pos){
        leftLatchServo.setPosition(pos);
        rightLatchServo.setPosition(pos + .05);
    }

    /**  GET ENCODER METHODS  */
    private double getAverageDriveTrainEncoder(MOVEMENT_DIRECTION movement_direction) {
        switch (movement_direction){
            case FORWARD: return getChangeInDriveTrainEncoder()[0];
            case STRAFE: return getChangeInDriveTrainEncoder()[1];
            case ROTATION: return getChangeInDriveTrainEncoder()[2];
        }

        return 0;
    }

    public double[] getChangeInDriveTrainEncoder() {
        return new double[]{
                (rightFrontDriveMotor.getCurrentPosition()),
                //(+ leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() + rightFrontDriveMotor.getCurrentPosition() + rightBackDriveMotor.getCurrentPosition()) / 4,
                //(- leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() + rightFrontDriveMotor.getCurrentPosition() - rightBackDriveMotor.getCurrentPosition()) / 4,
                (rightFrontDriveMotor.getCurrentPosition()),
                //(+ leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() - rightFrontDriveMotor.getCurrentPosition() - rightBackDriveMotor.getCurrentPosition()) / 4,
                (-rightFrontDriveMotor.getCurrentPosition())
        };
    }

    public int getLiftEncoder() {
        return liftMotor.getCurrentPosition();
    }

    public int getArmEncoder() {
        return armMotor.getCurrentPosition();
    }

    /**  STOP METHODS  */
    public void stopAllMotors(){
        lfMotorPower = 0;
        lbMotorPower = 0;
        rfMotorPower = 0;
        rbMotorPower = 0;

        leftIntakeMotor.setPower(0);
        rightIntakeMotor.setPower(0);
        liftMotor.setPower(0);
        armMotor.setPower(0);
    }

    //eliminates residual forces
    /*public void stopMotorsAndWait(double seconds) {
        mecanumPowerDrive(0, 0, 0);
        linearOpMode.sleep((int) (seconds * 1000));
    }*/

}