package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class OuttakeSlidesController {
    public enum ExtensionStatus{
        COMPACT,
        FAR,
        RUNTO,
        PRELOAD12,
        PRELOAD3
    }
    public ExtensionStatus currentStatus = ExtensionStatus.COMPACT;
    public ExtensionStatus previousStatus = null;

    public double compactPosition = 0.095; // De modificat
    public double farPosition = 0.39; // De modificat

    public double placePreloadPosition12 = 0.2;
    public double placePreloadPosition3 = 0.17;

    public Servo outtakeSlides = null;
    public double targetposition = compactPosition;
    public double previousPosition;
    public OuttakeSlidesController(RobotMap robot)
    {
        outtakeSlides= robot.outtakeSlides;
    }

    public void update()
    {
        if (currentStatus != previousStatus || previousPosition != targetposition)
        {
            previousStatus = currentStatus;
            previousPosition=targetposition;
            switch (currentStatus)
            {
                case COMPACT:
                {
                    outtakeSlides.setPosition(compactPosition);
                    break;
                }
                case FAR:
                {
                    outtakeSlides.setPosition(farPosition);
                    break;
                }
                case RUNTO:
                {
                    outtakeSlides.setPosition(targetposition);
                }
                case PRELOAD12:
                {
                    outtakeSlides.setPosition(placePreloadPosition12);
                }
                case PRELOAD3:
                {
                    outtakeSlides.setPosition(placePreloadPosition3);
                }
            }
        }
    }
}
