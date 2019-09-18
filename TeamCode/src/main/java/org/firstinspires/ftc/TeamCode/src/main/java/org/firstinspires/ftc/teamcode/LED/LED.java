package org.firstinspires.ftc.teamcode.LED;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class LED {

    Step[] flashPattern;
    int stepNumber;

    double timeOfLastPattern;
    RevBlinkinLedDriver.BlinkinPattern currentPattern;
    RevBlinkinLedDriver.BlinkinPattern oldPattern;

    //consturctor
    public LED(Step[] flashPattern, double time){
        this.flashPattern = flashPattern;
        start(time);
    }

    //method
    private void start(double time){
        /*a method that will repeat the same way in every constructor*/

        //starts at step 0, and sets the last pattern charge to the current time
        stepNumber = 0;
        timeOfLastPattern = time;

        //starts the pattern now, in case it is called
        currentPattern = flashPattern[0].getPattern();
    }


    public void update(double time){
        /*A method that changes depending on the pattern*/
        double timeSincePatternChange = time - timeOfLastPattern;

        //if the time since the pattern has changed is greater than or equal to the step l
        if(timeSincePatternChange >= flashPattern[stepNumber].getTime()){
            //the current Pattern will change
            oldPattern = currentPattern;

            //advance the step and set it too 0 if it too great
            stepNumber++;
            if(stepNumber == flashPattern.length) {stepNumber = 0;}

            //resets the time and the pattern
            timeOfLastPattern = time;
            currentPattern = flashPattern[stepNumber].getPattern();
        }
    }

    public void reduceLinear(double time){
        /*A method that reduces the amount of time for each step in the pattern*/
        for(int i = 0; i < flashPattern.length; i++){
            flashPattern[i].setTime(flashPattern[i].getTime() - time);
        }
    }
    public void reduceExponential(double decimal){
        /*A method that reduces the amount of time for each step in the pattern*/
        for(int i = 0; i < flashPattern.length; i++){
            flashPattern[i].setTime(flashPattern[i].getTime() * decimal);
        }
    }

    public RevBlinkinLedDriver.BlinkinPattern getPattern(){return currentPattern;}
    public boolean changed(){return oldPattern != currentPattern;}
}
