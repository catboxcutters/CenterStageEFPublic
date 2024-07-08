package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class SigurantaOuttakeController {
    public enum SigurantaOuttakeStatus{
        CLOSED,
        OPEN,
        RETARD
    }

    public SigurantaOuttakeStatus currentStatus = SigurantaOuttakeStatus.CLOSED;
    public SigurantaOuttakeStatus previousStatus = null;

    public double closedPosition = 0.5;
    public double openPosition = 0;
    public double retardPosition = 1;

    private Servo sigurantaOuttake = null;

    public SigurantaOuttakeController(RobotMap robot)
    {
        sigurantaOuttake = robot.sigurantaOuttake;
    }

    public void update()
    {
        if (currentStatus != previousStatus)
        {
            previousStatus = currentStatus;
            switch (currentStatus)
            {
                case CLOSED:
                {
                    sigurantaOuttake.setPosition(closedPosition);
                    break;
                }
                case OPEN:
                {
                    sigurantaOuttake.setPosition(openPosition);
                    break;
                }
                case RETARD:
                {
                    sigurantaOuttake.setPosition(retardPosition);
                    break;
                }
            }
        }
    }
}
