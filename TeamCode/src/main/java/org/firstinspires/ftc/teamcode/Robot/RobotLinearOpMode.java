package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Control.BallisticMotionProfile;
import org.firstinspires.ftc.teamcode.Control.EverHit;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.*;

public class RobotLinearOpMode extends Robot {

    private LinearOpMode linearOpMode;

    private double encodersToInches = 69/4000;
    private double inchesToEncoders = 4000/69;

    //now we make a variable to use later which represents the initial position when doing a runtoposition command
    private  double initialArmPosition = 0;

    //So im gald i got ur attention // heres why the lift code was broken: the bottom limit was set to 20000 not -20000. negative goes up on the lifter, and so the bottom limit is actually the top
    public BallisticMotionProfile liftProfile = new BallisticMotionProfile(0, 3300, 1000, 0.15, 1, .7);

    private double topArmEncoder = 2300;//I changed this
    private double bottomArmEncoder = 0;//and this. no underpass

    //I cranked up the decel distance so that it decelerates over a longer distance
    public BallisticMotionProfile armProfile = new BallisticMotionProfile(topArmEncoder, bottomArmEncoder, 1000, 0.2, 1, .7);


    private EverHit blockEverInIntake = new EverHit(false);

    //constructor
    public RobotLinearOpMode(LinearOpMode linearOpMode) {

        //creates the robot so that I can use all of the motors
        super(linearOpMode);

        //needs the linear opmode so that I can use telemetry and opModeIsActive()
        this.linearOpMode = linearOpMode;

        initIMU();
    }

    //sets the powers on either a power or velocity, and with variables or an array, where 0 is strafe, 1 is forward, and 2 is rotation
    //if I control it using an array, then it just sends it into an earlier method
    public void mecanumPowerDrive(double strafe, double forward, double rotation) {
        leftFrontDriveMotor.setPower(forward - strafe + rotation);
        leftBackDriveMotor.setPower(forward + strafe + rotation);
        rightFrontDriveMotor.setPower(forward + strafe - rotation);
        rightBackDriveMotor.setPower(forward - strafe - rotation);
    }

    //compensates for change in center of gravity with arm swinging behind the robot
    public void compensatedMecanumPowerDrive(double strafe, double forward, double rotation, double ratio){
        leftFrontDriveMotor.setPower(forward - strafe + rotation);
        leftBackDriveMotor.setPower(forward + strafe * ratio + rotation);
        rightFrontDriveMotor.setPower(forward + strafe - rotation);
        rightBackDriveMotor.setPower(forward - strafe * ratio - rotation);
    }

    public void mecanumPowerDrive(double[] controller) {
        mecanumPowerDrive(controller[0], controller[1], controller[2]);
    }

    public void mecVelocityDrive(double strafe, double forward, double rotation) {
        leftFrontDriveMotor.setVelocity(forward - strafe + rotation);
        leftBackDriveMotor.setVelocity(forward + strafe + rotation);
        rightFrontDriveMotor.setVelocity(forward + strafe - rotation);
        rightBackDriveMotor.setVelocity(forward - strafe - rotation);
    }

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

    public void mecVelocityDrive(double[] controller) {
        mecVelocityDrive(controller[0], controller[1], controller[2]);
    }

    public double[] getDriveTrainEncoders() {
        return new double[]{
                leftFrontDriveMotor.getCurrentPosition(),
                leftBackDriveMotor.getCurrentPosition(),
                rightFrontDriveMotor.getCurrentPosition(),
                rightBackDriveMotor.getCurrentPosition(),
        };
    }

    public double getAverageForwardDriveTrainEncoder(double[] startEncoderValues) {
        double averageEncoderPosition = 0;

        averageEncoderPosition += (leftFrontDriveMotor.getCurrentPosition() - startEncoderValues[0]);
        averageEncoderPosition += (leftBackDriveMotor.getCurrentPosition() - startEncoderValues[1]);
        averageEncoderPosition += (rightFrontDriveMotor.getCurrentPosition() - startEncoderValues[2]);
        averageEncoderPosition += (rightBackDriveMotor.getCurrentPosition() - startEncoderValues[3]);

        return averageEncoderPosition / 4;
    }



    public double getAverageStrafeDriveTrainEncoder(double[] startEncoderValues) {
        double averageEncoderPosition = 0;

        averageEncoderPosition -= (leftFrontDriveMotor.getCurrentPosition() - startEncoderValues[0]);
        averageEncoderPosition += (leftBackDriveMotor.getCurrentPosition() - startEncoderValues[1]);
        averageEncoderPosition += (rightFrontDriveMotor.getCurrentPosition() - startEncoderValues[2]);
        averageEncoderPosition -= (rightBackDriveMotor.getCurrentPosition() - startEncoderValues[3]);

        return averageEncoderPosition / 4;
    }

    public double getAverageRotationDriveTrainEncoder(double[] startEncoderValues) {
        double averageEncoderPosition = 0;

        averageEncoderPosition += (leftFrontDriveMotor.getCurrentPosition() - startEncoderValues[0]);
        averageEncoderPosition += (leftBackDriveMotor.getCurrentPosition() - startEncoderValues[1]);
        averageEncoderPosition -= (rightFrontDriveMotor.getCurrentPosition() - startEncoderValues[2]);
        averageEncoderPosition -= (rightBackDriveMotor.getCurrentPosition() - startEncoderValues[3]);
        return averageEncoderPosition / 4;
    }

    //two methods that turn precisely, which Casey made and I don't fully understand
    public void turnByDegree(double degree) {
        BallisticMotionProfile TurnProfile = new BallisticMotionProfile(0, 0, 90, .05, 1, .65);

        double currentAngle;
        double adjustedMotorPower;

        //gets the current angle from the IMU
        double startAngle = getRev2IMUAngle()[2];

        //calculates the angles the angle that it needs to be at, at the end
        double neededAngle = startAngle + degree;
        currentAngle = startAngle;

        //the loop that will run until the needed Angle is acheived
        if (currentAngle < neededAngle) {
            while ((currentAngle < neededAngle && linearOpMode.opModeIsActive())) {//will run until we get there
                currentAngle = getRev2IMUAngle()[2];////IMU something

                adjustedMotorPower = TurnProfile.RunToPositionWithAccel(startAngle, currentAngle, neededAngle);//get a rotation

                mecanumPowerDrive(0, 0, adjustedMotorPower);
            }
        } else if (currentAngle > neededAngle) {
            while ((currentAngle > neededAngle)  && linearOpMode.opModeIsActive()) {//will run until we get there
                currentAngle = getRev2IMUAngle()[2];////IMU something

                adjustedMotorPower = TurnProfile.RunToPositionWithAccel(startAngle, currentAngle, neededAngle);//get a rotation

                mecanumPowerDrive(0, 0, adjustedMotorPower);
            }
        }
    }

    public enum MOVEMENT_DIRECTION {
        STRAFE,
        FORWARD,
        ROTATION,
    }

    public void moveByInches(double desiredPositionChangeInInches, MOVEMENT_DIRECTION movement_direction, boolean withAccel) {

        double changeInAverageEncoderValue = 0;
        double desiredPositionChangeInEncoders = 0;
        double adjustedMotorPower = 0;
        double[] currentEncoderValues;

        BallisticMotionProfile DriveProfile = new BallisticMotionProfile(0, 0, 800, 0.05, 1, 0.75);

        desiredPositionChangeInEncoders = desiredPositionChangeInInches * inchesToEncoders;

        currentEncoderValues = getDriveTrainEncoders();

        do {
            if(movement_direction == STRAFE) changeInAverageEncoderValue = getAverageStrafeDriveTrainEncoder(currentEncoderValues);
            if(movement_direction == FORWARD) changeInAverageEncoderValue = getAverageForwardDriveTrainEncoder(currentEncoderValues);
            if(movement_direction == ROTATION) changeInAverageEncoderValue = getAverageRotationDriveTrainEncoder(currentEncoderValues);



            if(withAccel)adjustedMotorPower = DriveProfile.RunToPositionWithAccel(0, changeInAverageEncoderValue, desiredPositionChangeInEncoders);
            else adjustedMotorPower = DriveProfile.RunToPositionWithoutAccel(0, changeInAverageEncoderValue, desiredPositionChangeInEncoders);

            if(movement_direction == STRAFE) mecanumPowerDrive(adjustedMotorPower, 0, 0);
            if(movement_direction == FORWARD) mecanumPowerDrive(0, adjustedMotorPower, 0);
            if(movement_direction == ROTATION) mecanumPowerDrive(0,0, adjustedMotorPower);

            linearOpMode.telemetry.addData("Start Position:", 0);
            linearOpMode.telemetry.addData("Current Position:", changeInAverageEncoderValue);
            linearOpMode.telemetry.addData("End Position:", desiredPositionChangeInEncoders);
            linearOpMode.telemetry.update();

        } while((abs(desiredPositionChangeInEncoders - changeInAverageEncoderValue) > 50) && linearOpMode.opModeIsActive());
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

        } while((abs(desiredPositionChangeInEncoders) > abs(startEncoderValue - currentEncoderValue)) && linearOpMode.opModeIsActive());
    }

    public void moveToLatch(double desiredPositionChangeInInches) {

        double[] currentEncoderValues = new double[4];
        double currentAverageEncoderValue = 0;
        double desiredPositionChangeInEncoders = 0;
        double adjustedMotorPower = 0;

        BallisticMotionProfile DriveProfile = new BallisticMotionProfile(0, 0, 8 * inchesToEncoders, 0.05, 1, 0.75);

        desiredPositionChangeInEncoders = desiredPositionChangeInInches * inchesToEncoders;

        openLatch();

        currentEncoderValues = getDriveTrainEncoders();

        do {
            currentAverageEncoderValue = getAverageForwardDriveTrainEncoder(currentEncoderValues);

            adjustedMotorPower = DriveProfile.RunToPositionWithAccel(0, currentAverageEncoderValue, desiredPositionChangeInEncoders);

            mecanumPowerDrive(0, adjustedMotorPower, 0);

        } while((abs(desiredPositionChangeInEncoders - currentAverageEncoderValue) > 50) && linearOpMode.opModeIsActive());

        closeLatch();
    }

    public void moveToStone(double maxBotPower, double intakePower, double extraEncoderDistance) {

        double[] currentEncoderValues = new double[4];
        double currentAverageEncoderValue = 0;
        double desiredPositionChangeInEncoders = 1000000000;
        double adjustedMotorPower = 0;
        boolean hasFoundBlock = false;

        BallisticMotionProfile DriveProfile = new BallisticMotionProfile(0, 0, 800, 0.05, 1, maxBotPower);

        blockHasLeftIntake();

        currentEncoderValues = getDriveTrainEncoders();

        do {
            currentAverageEncoderValue = getAverageForwardDriveTrainEncoder(currentEncoderValues);

            adjustedMotorPower = DriveProfile.RunToPositionWithAccel(0, currentAverageEncoderValue, desiredPositionChangeInEncoders);

            mecanumPowerDrive(0, adjustedMotorPower, 0);

            setIntakePower(intakePower);

        } while(!isBlockInIntake() && linearOpMode.opModeIsActive());

    }

    //eliminates residual forces
    public void stopMotorsAndWait(double seconds) {
        mecanumPowerDrive(0, 0, 0);
        linearOpMode.sleep((int) (seconds * 1000));
    }

    //sets the intake power and velocity
    public void setIntakePower(double intakePower) {
        if (intakeIsOpen()) {
            rightIntakeMotor.setPower(intakePower);
            leftIntakeMotor.setPower(-intakePower);
        } else {
            rightIntakeMotor.setPower(intakePower);
            leftIntakeMotor.setPower(intakePower);
        }

    }

    public void setIntakeVelocity(double intakeVelocity) {
        if (intakeIsOpen()) {
            rightIntakeMotor.setVelocity(intakeVelocity);
            leftIntakeMotor.setVelocity(-intakeVelocity);
        } else {
            rightIntakeMotor.setVelocity(intakeVelocity);
            leftIntakeMotor.setVelocity(intakeVelocity);
        }
    }

    //sets the lift power and veloctiy
    public void setLiftPower(double liftPower) {
        liftMotor.setPower(liftPower);
    }

    public int getLiftEncoder() {
        return liftMotor.getCurrentPosition();
    }

    //sets the arm power and velocity
    public void setArmPower(double armPower) {
        armMotor.setPower(armPower);
    }

    public int getArmEncoder() {
        return armMotor.getCurrentPosition();
    }

    //tells us if the arm is on targetEncoder
    public boolean armHasArrived(double targetEncoder) {
        boolean arrivedAtTargetEncoder;

        //if the target encoder position is below the very starting one, and the current position is below that
        if ((initialArmPosition < targetEncoder) && (targetEncoder < getArmEncoder())) {
            arrivedAtTargetEncoder = true;
        }
        //the exact opposite condidion that yields essentially the same result
        else if ((initialArmPosition > targetEncoder) && (targetEncoder > getArmEncoder())) {
            arrivedAtTargetEncoder = true;
        } else {
            arrivedAtTargetEncoder = false;
        }

        return arrivedAtTargetEncoder;
    }

    public void openGrabber() {
        blockGrabberServo.setPosition(.7);
    }

    public void closeGrabber() {
        blockGrabberServo.setPosition(.3);
    }

    public void dropMarker() {
        markerLatchServo.setPosition(1);
    }

    public void holdMarker() {
        markerLatchServo.setPosition(0);
    }

    public void closeLatch() {
        leftLatchServo.setPosition(.3);
        rightLatchServo.setPosition(.45);
    }

    public void openLatch() {
        leftLatchServo.setPosition(.5);
        rightLatchServo.setPosition(.65);
    }

    public boolean isBlockInIntake(){
        blockEverInIntake.update(blockInIntake());
        return blockEverInIntake.wasEverHit();
    }

    public void blockHasLeftIntake(){
        blockEverInIntake.reset();
    }
}