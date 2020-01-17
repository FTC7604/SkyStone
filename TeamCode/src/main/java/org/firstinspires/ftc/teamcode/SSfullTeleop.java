//imports the package so that all the code can be used
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.BallisticMotionProfile;
import org.firstinspires.ftc.teamcode.Control.HumanController;
import org.firstinspires.ftc.teamcode.Control.Toggle;

import static java.lang.Math.abs;

@TeleOp(name = "Skystone Main Teleop", group = "Linear Opmode")
public class SSfullTeleop extends LinearOpMode {

    //private final double LIFT_HOME_POSITION = -20;//for controlling the lifter
    private final double WEIGHT_COMP_RATIO = 1.15;
    private final double ARM_HOME_POSITION = 0;
    private final double ARM_SCORING_POSITION = 2300;
    private double intakeCorrectionStartTime = -200;
    private double[] driveTrainController = new double[3];
    private BallisticMotionProfile liftProfile = new BallisticMotionProfile(-1300, -10, 500, 0.25, 1, .7);
    private BallisticMotionProfile armProfile = new BallisticMotionProfile(3300, 0, 1000, 0.3, 1, 1);
    private double intakePower = 0;
    private double armPower = 0;
    private double liftPower = 0;
    private double armPosition = 0;
    private double liftPosition = 0;
    //private double liftHoldposition = 0;
    private double armHoldPosition = 0;
    private boolean armGoingToScoringPosition = false;
    private boolean armGoingToHomePosition = false;
    private double initialArmPosition = 0;
    private HumanController humanController = new HumanController(0.1, 1);
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFrontDriveMotor;
    private DcMotor leftFrontDriveMotor;
    private DcMotor rightBackDriveMotor;
    RevBlinkinLedDriver blinkin;
    /*private void holdLiftUp() {
        if (liftPosition > liftHoldposition) {
            liftMotor.setPower(-0.4);
        }
        else {
            liftMotor.setPower(0);
        }
    }*/
    private DcMotor leftBackDriveMotor;
    private DcMotor rightIntakeMotor;
    private DcMotor leftIntakeMotor;
    private DcMotor armMotor;
    private DcMotor liftMotor;
    private Servo leftLatchServo;
    private Servo rightLatchServo;
    private Servo blockGrabberServo;
    private Servo markerLatchServo;
    private DigitalChannel intakeFull;
    private DigitalChannel openIntakeTouchSensor;
    private Toggle latchIsDown = new Toggle(false);
    private Toggle grabberIsEngaged = new Toggle(false);
    private Toggle driveMode = new Toggle(false);
    private Toggle markerDropper = new Toggle(true);

    @Override
    public void runOpMode() {

        InitHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        openGrabber();

        while (opModeIsActive()) {

            runDrive();

            runLifter();

            runArm();

            runIntake();

            runServos();

            getPositionTargets();

            sendDriverFeedback();
        }
    }
    //private boolean liftGoingToHomePosition = false;//added new

    //This is the LED method
    private void sendDriverFeedback() {

        //TODO the main thing we want to get feedback on is the latch, though I have looped in something with the intake sensor so that we know if we have a stone.
        if (!intakeFull.getState()) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
            //LEDs flashing bright or something
        } else if (latchIsDown.get()) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
            //LEDS a certain color to indicate the latch is down
        } else {
            //LEDs some other way, maybe an alliance color if we want
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }

        //Well get rid of this telemetry for lag reduction
        telemetry.addData("Arm Position: ", armPosition);
        telemetry.addData("Lift Position: ", liftPosition);
        telemetry.update();
    }

    //tells us if the arm is on target
    private boolean armHasArrived(double current, double target) {
        boolean arrived;

        //make sure we made it depending on which way we came
        if ((initialArmPosition <= target) && (target <= current)) {
            arrived = true;
        } else
            arrived = (initialArmPosition >= target) && (target >= current);
        return arrived;
    }
    //private double initialLiftPosition = 0;

    private void InitHardware() {
        //assigns it to the config
        rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "lf");
        leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "rf");
        rightBackDriveMotor = hardwareMap.get(DcMotor.class, "lb");
        leftBackDriveMotor = hardwareMap.get(DcMotor.class, "rb");
        rightIntakeMotor = hardwareMap.get(DcMotor.class, "ri");
        leftIntakeMotor = hardwareMap.get(DcMotor.class, "li");
        armMotor = hardwareMap.get(DcMotor.class, "ax");
        liftMotor = hardwareMap.get(DcMotor.class, "lx");
        leftLatchServo = hardwareMap.get(Servo.class, "ll");
        rightLatchServo = hardwareMap.get(Servo.class, "rl");
        blockGrabberServo = hardwareMap.get(Servo.class, "bg");
        markerLatchServo = hardwareMap.get(Servo.class, "ml");
        intakeFull = hardwareMap.get(DigitalChannel.class, "bt");
        openIntakeTouchSensor = hardwareMap.get(DigitalChannel.class, "it");
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "bk");

        //sets the direction
        rightFrontDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        rightIntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        leftIntakeMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftLatchServo.setDirection(Servo.Direction.FORWARD);
        rightLatchServo.setDirection(Servo.Direction.REVERSE);
        intakeFull.setMode(DigitalChannel.Mode.INPUT);
        openIntakeTouchSensor.setMode(DigitalChannel.Mode.INPUT);

        //Brake
        rightFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Encoders
        rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void closeGrabber() {
        if (!grabberIsEngaged.get()) grabberIsEngaged.update(true);
        blockGrabberServo.setPosition(0);
    }

    private void openGrabber() {
        if (grabberIsEngaged.get()) grabberIsEngaged.update(true);
        blockGrabberServo.setPosition(.7);
    }

    private void dropMarker() {
        markerLatchServo.setPosition(1);
    }

    private void holdMarker() {
        markerLatchServo.setPosition(0);
    }

    private void closeLatch() {
        leftLatchServo.setPosition(.3);
        rightLatchServo.setPosition(.45);
    }

    private void openLatch() {
        leftLatchServo.setPosition(.5);
        rightLatchServo.setPosition(.65);
    }

    private void mecanumPowerDrive(MOVEMENT_DIRECTION movement_direction, double power) {
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

    private void mecanumPowerDrive(double strafe, double forward, double rotation) {
        leftFrontDriveMotor.setPower(forward - strafe + rotation);
        leftBackDriveMotor.setPower(forward + strafe + rotation);
        rightFrontDriveMotor.setPower(forward + strafe - rotation);
        rightBackDriveMotor.setPower(forward - strafe - rotation);
    }

    //compensates for change in center of gravity with arm swinging behind the robot
    private void compensatedMecanumPowerDrive(double strafe, double forward, double rotation, double ratio) {
        leftFrontDriveMotor.setPower(forward - strafe + rotation);
        leftBackDriveMotor.setPower(forward + strafe * ratio + rotation);
        rightFrontDriveMotor.setPower(forward + strafe - rotation);
        rightBackDriveMotor.setPower(forward - strafe * ratio - rotation);
    }

    private void mecanumPowerDrive(double[] controller) {
        mecanumPowerDrive(controller[0], controller[1], controller[2]);
    }

    private void setIntakePower(double intakePower) {
        if (!openIntakeTouchSensor.getState()) {//jammed
            rightIntakeMotor.setPower(intakePower);
            leftIntakeMotor.setPower(1);
            intakeCorrectionStartTime = runtime.milliseconds();
        } else if (runtime.milliseconds() < intakeCorrectionStartTime + 100) {//unjammed but still needs fixing
            rightIntakeMotor.setPower(intakePower);
            leftIntakeMotor.setPower(1);
        } else {//all clear
            rightIntakeMotor.setPower(intakePower);
            leftIntakeMotor.setPower(intakePower);
        }

    }

    private void runDrive() {
        //sets up the condidtion for the drivetrain
        driveMode.update(gamepad1.right_bumper);

        if (driveMode.get()) {//tank
            driveTrainController[1] = humanController.linearDriveProfile(((-gamepad1.right_stick_y) * (abs(-gamepad1.right_stick_y)) + ((-gamepad1.left_stick_y) * (abs(-gamepad1.left_stick_y)))) / 2);
            driveTrainController[0] = humanController.linearDriveProfile(-(((-gamepad1.right_stick_x) * (abs(-gamepad1.right_stick_x)) + ((-gamepad1.left_stick_x) * (abs(-gamepad1.left_stick_x)))) / 2));
            driveTrainController[2] = humanController.linearDriveProfile(((-gamepad1.right_stick_y) - (-gamepad1.left_stick_y)) / 2);
        } else {//normal
            driveTrainController[1] = humanController.linearDriveProfile(-gamepad1.left_stick_y);
            driveTrainController[0] = humanController.linearDriveProfile(gamepad1.left_stick_x);
            driveTrainController[2] = humanController.linearDriveProfile(-gamepad1.right_stick_x);
        }

        if (gamepad1.left_bumper) {//now it is a reverse half power mode.

            driveTrainController[1] /= 3;///should be a bit slower if these changes work
            driveTrainController[0] /= 2;
            driveTrainController[2] /= 3;
            compensatedMecanumPowerDrive(driveTrainController[0], driveTrainController[1], driveTrainController[2], WEIGHT_COMP_RATIO);

            //I am trying a different drive mode without weight comp

        } else {
            mecanumPowerDrive(driveTrainController);
        }
    }

    private void runLifter() {
        liftPower = liftProfile.limitWithoutAccel(liftMotor.getCurrentPosition(), -gamepad2.right_stick_y);
        liftPosition = liftMotor.getCurrentPosition();
        liftMotor.setPower(liftPower);
    }

    private void runArm() {///TODO somethings up here, you need to rewrite the whole thing
        armPosition = armMotor.getCurrentPosition();


        if (armGoingToScoringPosition) {
            //go to scoring
            if (armHasArrived(armPosition, ARM_SCORING_POSITION)) {
                armHoldPosition = armPosition;
                armGoingToScoringPosition = false;
                armPower = 0;
                //reset hold point
                //stop trying to go there
            } else {
                armPower = armProfile.RunToPositionWithAccel(initialArmPosition, armPosition, ARM_SCORING_POSITION);
            }
        } else if ((liftPower != 0) && armPosition < 200 && gamepad2.left_stick_y <= 0) {
            armPower = 0.1;
        } else if (armGoingToHomePosition) {
            //go to home
            if (armHasArrived(armPosition, ARM_HOME_POSITION)) {
                armHoldPosition = armPosition;
                armGoingToHomePosition = false;
                armPower = 0;
                //reset hold point
                //stop trying to go there
            } else {
                armPower = armProfile.RunToPositionWithAccel(initialArmPosition, armPosition, ARM_HOME_POSITION);
            }

        } else {
            if (Math.abs(gamepad2.left_stick_y) < 0.1) {
                holdArmUp();
            } else {
                armHoldPosition = armPosition;
                armPower = armProfile.limitWithoutAccel(armPosition, gamepad2.left_stick_y);
            }
        }


        armMotor.setPower(armPower);
    }

    private void getPositionTargets() {
        //this code checks to see if we are going to a new target, and of so changes the desired direction and resets the initial position
        if (gamepad2.left_stick_button) {
            armGoingToScoringPosition = false;
            armGoingToHomePosition = false;
        } else if (gamepad2.dpad_down) {//this one makes the arm go up
            armGoingToScoringPosition = true;
            armGoingToHomePosition = false;
            //liftGoingToHomePosition = false;
            initialArmPosition = armMotor.getCurrentPosition();
        } else if (gamepad2.dpad_up) {//this one makes the lifter go down and the arm go home (basically a reset)
            armGoingToHomePosition = true;
            //liftGoingToHomePosition = true;
            armGoingToScoringPosition = false;
            initialArmPosition = armMotor.getCurrentPosition();
            //initialLiftPosition = liftMotor.getCurrentPosition();
        }
    }

    private void runIntake() {
        intakePower = gamepad2.right_trigger - gamepad2.left_trigger;
        setIntakePower(intakePower);
    }

    private void runServos() {
        latchIsDown.update(gamepad2.x);
        markerDropper.update(gamepad2.a);
        //grabberIsEngaged.update(gamepad2.right_bumper);

        //peter gets full control
        if (armGoingToHomePosition) openGrabber();
        else if (gamepad2.y) closeGrabber();
        else if (gamepad2.right_bumper) openGrabber();

        if (!latchIsDown.get()) openLatch();
        else closeLatch();

        if (markerDropper.get()) holdMarker();
        else dropMarker();
    }

    private void holdArmUp() {
        if (armPosition > armHoldPosition) {
            armPower = -0.4;
        } else {
            armPower = 0;
        }
    }
    private enum MOVEMENT_DIRECTION {
        STRAFE,
        FORWARD,
        ROTATION,
    }
}