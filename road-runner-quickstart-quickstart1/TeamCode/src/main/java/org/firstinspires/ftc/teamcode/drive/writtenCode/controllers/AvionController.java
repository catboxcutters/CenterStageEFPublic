package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.code.Attribute;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class AvionController {
    public enum LaunchStatus{

        LAUNCH,

        INIT,



    }
    public LaunchStatus currentStatus = AvionController.LaunchStatus.INIT;
    public LaunchStatus previousStatus = null;

    public double initPosition = 0;
    public double launchPosition = 0.7;
    private Servo airplaneServo = null;
    public AvionController(RobotMap robot)
    {
        airplaneServo = robot.airplaneServo;
    }
    public void update()
    {
        if (currentStatus != previousStatus)
        {
            previousStatus = currentStatus;
            switch (currentStatus)
            {
                case INIT:
                {
                    airplaneServo.setPosition(initPosition);
                    break;
                }
                case LAUNCH:
                {
                    airplaneServo.setPosition(launchPosition);
                    break;
                }
            }
            }
        }
}
