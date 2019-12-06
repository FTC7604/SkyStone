//imports the package so that all the code can be used
package org.firstinspires.ftc.teamcode;

//all the import

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//allows me to use the package code
import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;
import org.firstinspires.ftc.teamcode.Control.BallisticMotionProfile;
import org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode;

//imports all of the math function
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.STRAFE;

//name that appears on the phone and the group that it is a part of
@Autonomous(name = "Rubie Test OpMode", group = "Linear Opmode")

//class name that corresponds with the file, it is a linear opMode that I then changed
public class rubieAutoPaths extends LinearOpMode {


    /*Mini Lesson:
    Public - Can be accessed from anywhere
    Private - Can be accessed from within the class
    *Nothing* - Can be accessed from within the package (package private)
    Proteccted - Subclasses can access the object (think extends)
     */

    //the amount of time that the program has run
    private ElapsedTime runtime = new ElapsedTime();

    /*Mini Lesson:
    Int - Whole numbers
    Long - Long whole numbers
    Float - Whole numbers but hexidecimal(base 16)
    Double - Floating point (decimal values)
    Char - Character, the basis of a word
    Boolean - True/False
     */

    //so these exist so that I do not get confused, not really necessary
    double[] driveTrainController = new double[3];
    double intakePower = 0;
    double armPower = 0;
    double liftPower = 0;

    //sensor values, also exist to make the code cleaner
    double armPosition = 0;
    boolean blockIntakeTouchSensor = true;

    //this is the loop that repeats until the end of teleOp.
    @Override
    public void runOpMode() {

        RobotLinearOpMode robotLinearOpMode = new RobotLinearOpMode(this);

        robotLinearOpMode.setDriveTrainRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotLinearOpMode.setDriveTrainZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);

        robotLinearOpMode.setIntakeRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotLinearOpMode.setIntakeZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);

        robotLinearOpMode.setArmRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotLinearOpMode.setArmZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //this is when the robot receives the 'play' command
        waitForStart();
        runtime.reset();

        double waitTime;


        robotLinearOpMode.moveByInches(12, FORWARD);

        waitTime = 1000000000;

        if (robotLinearOpMode.blockInIntake())
            if (waitTime > System.currentTimeMillis())
                robotLinearOpMode.setIntakePower(1);


        robotLinearOpMode.closeGrabber();

        robotLinearOpMode.moveByInches(-12, FORWARD);

        robotLinearOpMode.turnByDegree(90.0);

        robotLinearOpMode.moveByInches(48, FORWARD);

        robotLinearOpMode.getArmEncoder();
        if(robotLinearOpMode.getArmEncoder()< 30000)
            robotLinearOpMode.setArmPower(1);

        robotLinearOpMode.openGrabber();

        robotLinearOpMode.getArmEncoder();
        if(robotLinearOpMode.getArmEncoder()> 0)
            robotLinearOpMode.setArmPower(-1);

        



    }
}


//    enum AUTOPATHNAMES {
//    R1FOUNDATIONPARKR, R1FOUNDATIONSTONEPARKR, R1FOUNDATIONSTONESTONE2PARKR, R1FOUNDATIONPARKL, R1FOUNDATIONSTONEPARKL, R1FOUNDATIONSTONESTONE2PARKL,
//    R1PARKL, R1PARKR, R1STONEFOUNDATIONPARKL, R1STONEFOUNDATIONPARKR, R1STONESTONE2FOUNDATIONPARKL, R1STONESTONE2FOUNDATIONPARKR, R1STONEPARKL, R1STONEPARKR,
//    R1STONESTONE2PARKL, R1STONESTONE2PARKR,
//        R2FOUNDATIONPARKR, R2FOUNDATIONSTONEPARKR, R2FOUNDATIONSTONESTONE2PARKR, R2FOUNDATIONPARKL, R2FOUNDATIONSTONEPARKL, R2FOUNDATIONSTONESTONE2PARKL,
//        R2PARKL, R2PARKR, R2STONEFOUNDATIONPARKL, R2STONEFOUNDATIONPARKR, R2STONESTONE2FOUNDATIONPARKL, R2STONESTONE2FOUNDATIONPARKR, R2STONEPARKL, R2STONEPARKR,
//        R2STONESTONE2PARKL, R2STONESTONE2PARKR,
//       NR1FOUNDATIONPARKR,NR1FOUNDATIONSTONEPARKR,NR1FOUNDATIONSTONESTONE2PARKR,NR1FOUNDATIONPARKL,NR1FOUNDATIONSTONEPARKL,NR1FOUNDATIONSTONESTONE2PARKL,
//       NR1PARKL,NR1PARKR,NR1STONEFOUNDATIONPARKL,NR1STONEFOUNDATIONPARKR,NR1STONESTONE2FOUNDATIONPARKL,NR1STONESTONE2FOUNDATIONPARKR,NR1STONEPARKL,NR1STONEPARKR,
//       NR1STONESTONE2PARKL,NR1STON
//                NR2FOUNDATIONPARKR,NR2FOUNDATIONSTONEPARKR,NR2FOUNDATIONSTONESTONE2PARKR,NR2FOUNDATIONPARKL,NR2FOUNDATIONSTONEPARKL,NR2FOUNDATIONSTONESTONE2PARKL,
//                NR2PARKL,NR2PARKR,NR2STONEFOUNDATIONPARKL,NR2STONEFOUNDATIONPARKR,NR2STONESTONE2FOUNDATIONPARKL,NR2STONESTONE2FOUNDATIONPARKR,NR2STONEPARKL,NR2STONEPARKR,
//                NR2STONESTONE2PARKL,NR2STONESTONE2PARKR,
//
//    }


/* foundation from start (starting facing the wall)
        back up ~2 feet
        latch
        strafe to right until foundation hits wall
        start next thing
 */

/* foundation from stone ()*/

/*
boolean lastGamepad1a = false;
        boolean lastGamepad1b = false;

        while (opModeIsActive()) {
            // enter using lb, exit using rb
            // ++rt --lt
            telemetry.addData("Status", "auxiliary mode: " + mode.toString());
            telemetry.addData("Status", "level " + stackingLevel);
            telemetry.update();

            if (gamepad2.left_bumper)
                mode = TeleStackingMode.stacking; // this enters the stacking mode

            if (gamepad2.right_bumper)
                mode = TeleStackingMode.normal; // this exits the stacking mode


            {
                boolean dpadUp = gamepad2.dpad_up;
                if (dpadUp != lastGamepad1a && dpadUp)
                    stackingLevel = (stackingLevel < 3 ? stackingLevel + 1 : 3);
                lastGamepad1a = dpadUp;
            } // if up is pressed, higher level
            {
                boolean dpadDown = gamepad2.dpad_down;
                if (dpadDown != lastGamepad1b && dpadDown)
                    stackingLevel = (stackingLevel > 0 ? stackingLevel - 1 : 0);
                lastGamepad1b = dpadDown;
            } // if down is pressed, lower level


            switch (mode) {
                default:
                    mode = TeleStackingMode.normal;
                    //  fall through
                case normal:
                    //  do nothing
                    break;
                case stacking:


                    //  prepare for stacking
                    switch (stackingLevel) {
                        default:
                            stackingLevel = 0;
                            //  fall through
                        case 0:
                            robotLinearOpMode.closeGrabber();
                            timeoutReset();
                            stackingLevel++;
                            break;
                        case 1:
                            if (timeoutHasBeenAtLeast(timeOutDurationForGrabber)) {
                                stackingLevel++;
                                robotLinearOpMode.openGrabber();
                                timeoutReset();
                            }

                            break;
                        case 2:
                            if (timeoutHasBeenAtLeast(timeOutDurationForGrabber)) {
                                stackingLevel = 0;


                            }
                            break;
                        case 3:
                            break;

                        case 4:
                            break;

                        case 5:
                            break;

                        case 6:
                            break;

                        case 7:
                            break;

                        case 8:
                            break;

                        case 9:
                            break;

                        case 10:
                            break;
                    }
                    break;
            }
        }
    }

    private enum TeleStackingMode {
        normal,
        stacking;
    }

    private void timeoutReset() {
        t0 = System.currentTimeMillis();
    }

    private boolean timeoutHasBeenAtLeast(long ms) {
        return (System.currentTimeMillis() - t0) >= ms;
    }

<<<<<<< HEAD
    public void goUpStackingLevel() {

         //fixme ask about this method
    }


=======
>>>>>>> parent of 27713c7... Has fixme
    public void armGoesOverTheSame() {


    }

    private long t0 = System.currentTimeMillis();

    private static final long timeOutDurationForGrabber = 3000;

    private TeleStackingMode mode = TeleStackingMode.normal;
    private int stackingLevel = 0;
}

    }
 */




