//imports the package so that all the code can be used
package org.firstinspires.ftc.teamcode;


//all the imports that come from qualcomm, mainly needing to do with the Hardware and the phone running
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//all the imports that come from Casey and I in the Package
import org.firstinspires.ftc.teamcode.Control.BallisticMotionProfile;
import org.firstinspires.ftc.teamcode.Control.EverHit;
import org.firstinspires.ftc.teamcode.Control.HumanController;
import org.firstinspires.ftc.teamcode.Control.Toggle;
import org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode;

//all the imports that come from java, abs is absolute value
import static java.lang.Math.abs;

//name that appears on the phone and the group that it is a part of
@TeleOp(name = "Test OpMode", group = "Linear Opmode")

//class name that corresponds with the file, it is a linear opMode that I then changed
public class testOpMode extends LinearOpMode {


    /*Mini Lesson:
    Public - Can be accessed from anywhere
    Private - Can be accessed from within the class
    *Nothing* - Can be accessed from within the package (package private)
    Proteccted - Subclasses can access the object (think extends)
     */

    //We start with some arm positions that we will go to in the future
    //this one is where we start, with the arm at 0 resting in the robot.
    private final double ARM_DOWN_POSITION = 0;
    //this one is all
    private final double ARM_UP_POSITION = 1600;
    //so these exist so that I do not get confused, not strictly necessary, 0 is the strafe, 1 is the forward, and 2 is the rotation
    private double[] driveTrainController = new double[3];
    /*Mini Lesson:
    Int - Whole numbers
    Long - Long whole numbers
    Float - Whole numbers but hexidecimal(base 16)
    Double - Floating point (decimal values)
    Char - Character, the basis of a word
    Boolean - True/False
     */
    private double intakePower = 0;
    private double armPower = 0;
    private double liftPower = 0;

    /////////////////Casey's Position Shenanegins - making some runtoposition commands
    //sensor values, also exist to make the code cleaner
    private double armPosition = 0;
    private double liftPosition;
    //these are used to determine whether or not we are actively trying to go to a certain preset position
    //as determined by user input
    private boolean armGoingToUpPosition = false;
    private boolean armGoingToDownPosition = false;

    /////////////////////////

    private boolean blockIntakeTouchSensor;
    private RobotLinearOpMode robotLinearOpMode;
    private EverHit blockEverInIntake;
    private double topArmEncoder = 2300;//I changed this
    private double bottomArmEncoder = 0;//and this. no underpass
    private HumanController humanController = new HumanController(0.1, 1);
    //the amount of time that the program has run
    private ElapsedTime runtime = new ElapsedTime();

    //this is the loop that repeats until the end of teleOp.
    @Override
    public void runOpMode() {
        robotLinearOpMode = new RobotLinearOpMode(this);

        Toggle latchIsDown = new Toggle(false);
        Toggle grabberIsEngaged = new Toggle(false);

        Toggle driveMode = new Toggle(false);
        Toggle markerDropper = new Toggle(true);
        blockEverInIntake = new EverHit();

        //So im gald i got ur attention // heres why the lift code was broken: the bottom limit was set to 20000 not -20000. negative goes up on the lifter, and so the bottom limit is actually the top
        BallisticMotionProfile liftProfile = new BallisticMotionProfile(0, 4700, 1000, 0.1, 1, .7);

        //I cranked up the decel distance so that it decelerates over a longer distance
        BallisticMotionProfile armProfile = new BallisticMotionProfile(topArmEncoder, bottomArmEncoder, 1000, 0.1, 1, .7);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //this is when the robot receives the 'play' command
        waitForStart();
        runtime.reset();

        robotLinearOpMode.setAllMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotLinearOpMode.setAllMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotLinearOpMode.setArmRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotLinearOpMode.setLiftRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotLinearOpMode.setAllMotorZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeIsActive()) {

            //sets up the condidtion for the drivetrain
            driveMode.update(gamepad1.right_bumper);


            if (driveMode.get()) {
                driveTrainController[1] = humanController.linearDriveProfile(((-gamepad1.right_stick_y) * (abs(-gamepad1.right_stick_y)) + ((-gamepad1.left_stick_y) * (abs(-gamepad1.left_stick_y)))) / 2);
                driveTrainController[0] = humanController.linearDriveProfile(-(((-gamepad1.right_stick_x) * (abs(-gamepad1.right_stick_x)) + ((-gamepad1.left_stick_x) * (abs(-gamepad1.left_stick_x)))) / 2));
                driveTrainController[2] = humanController.linearDriveProfile(((-gamepad1.right_stick_y) - (-gamepad1.left_stick_y)) / 2);
            } else {
                driveTrainController[1] = humanController.linearDriveProfile(-gamepad1.left_stick_y);
                driveTrainController[0] = humanController.linearDriveProfile(gamepad1.left_stick_x);
                driveTrainController[2] = humanController.linearDriveProfile(-gamepad1.right_stick_x);
            }

            if (gamepad1.left_bumper) {
                driveTrainController[1] /= 2;
                driveTrainController[0] /= 2;
                driveTrainController[2] /= 2;
            }

            //increments the intake power
            intakePower = gamepad2.right_trigger - gamepad2.left_trigger;

            ///////////The second part of Casey's arm thing

            if (armGoingToUpPosition) {

                if (robotLinearOpMode.armHasArrived( ARM_UP_POSITION)) {
                    armGoingToUpPosition = false;
                    armPower = 0;
                } else
                    armPower = armProfile.RunToPositionWithAccel(robotLinearOpMode.initialArmPosition, robotLinearOpMode.getArmEncoder(), ARM_UP_POSITION);

            } else if (armGoingToDownPosition) {

                if (robotLinearOpMode.armHasArrived(ARM_DOWN_POSITION)) {
                    armGoingToDownPosition = false;
                    armPower = 0;
                } else
                    armPower = armProfile.RunToPositionWithAccel(robotLinearOpMode.initialArmPosition, robotLinearOpMode.getArmEncoder(), ARM_DOWN_POSITION);

            } else {
                armPower = armProfile.V2limitWithAccel(robotLinearOpMode.getArmEncoder(), gamepad2.left_stick_y);
            }

            //this code checks to see if we are going to a new target, and of so changes the desired direction and resets the initial position
            if (gamepad2.dpad_down) {
                armGoingToUpPosition = true;
                armGoingToDownPosition = false;
                robotLinearOpMode.initialArmPosition = robotLinearOpMode.getArmEncoder();
            } else if (gamepad2.dpad_up) {
                armGoingToDownPosition = true;
                armGoingToUpPosition = false;
                robotLinearOpMode.initialArmPosition = robotLinearOpMode.getArmEncoder();
            }

            //////////////////End Casey's thing


            //this should work but be careful
            liftPower = liftProfile.V2limitWithAccel(robotLinearOpMode.getLiftEncoder(), gamepad2.right_stick_y);
            //liftPower = gamepad2.left_stick_y / 2;

            latchIsDown.update(gamepad2.x);
            grabberIsEngaged.update(gamepad2.y);
            markerDropper.update(gamepad2.a);

            blockEverInIntake.update(robotLinearOpMode.blockInIntake());

            if (grabberIsEngaged.get()) robotLinearOpMode.closeGrabber();
            else robotLinearOpMode.openGrabber();

            if (latchIsDown.get()) robotLinearOpMode.closeLatch();
            else robotLinearOpMode.openLatch();

            if (markerDropper.get()) robotLinearOpMode.holdMarker();
            else robotLinearOpMode.dropMarker();

            //sends this to the motors.
            robotLinearOpMode.mecanumPowerDrive(driveTrainController);
            robotLinearOpMode.setIntakePower(intakePower);
            robotLinearOpMode.setArmPower(armPower);
            robotLinearOpMode.setLiftPower(liftPower);

            //gets the position arm encoder
            armPosition = robotLinearOpMode.getArmEncoder();
            liftPosition = robotLinearOpMode.getLiftEncoder();
            blockIntakeTouchSensor = robotLinearOpMode.blockInIntake();

            sendTelemetry();
        }
    }

    void sendTelemetry() {
        telemetry.addData("Arm Position: ", armPosition);
        telemetry.addData("Lift Position: ", liftPosition);
        telemetry.addData("Block Intake Touch Boolean: ", blockEverInIntake.isHit());
        telemetry.addData("Open Intake Touch Boolean", robotLinearOpMode.intakeIsOpen());
        telemetry.update();
    }
}

