package org.firstinspires.ftc.teamcode.LED;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class LedPatternStep {
    private RevBlinkinLedDriver.BlinkinPattern stepPattern;
    private double timeStep;

    public LedPatternStep(RevBlinkinLedDriver.BlinkinPattern pattern, double time){
        this.stepPattern = pattern;
        this.timeStep = time;
    }

    RevBlinkinLedDriver.BlinkinPattern getPattern(){return stepPattern;}
    double getTime(){return timeStep;}
    void setTime(double time){timeStep = time;}
}
