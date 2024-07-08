package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class ParbrizController {
    public enum ParbrizStatus{
        CLOSED,
        OPEN,
    }

    public ParbrizStatus currentStatus = ParbrizStatus.CLOSED;
    public ParbrizStatus previousStatus = null;

    public double closedPosition = 0.5; // De modificat
    public double openPosition = 0; // De modificat

    private Servo parbrizOuttake = null;

    public ParbrizController(RobotMap robot)
    {
        parbrizOuttake = robot.parbrizOuttake;
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
                    parbrizOuttake.setPosition(closedPosition);
                    break;
                }
                case OPEN:
                {
                    parbrizOuttake.setPosition(openPosition);
                    break;
                }
            }
        }
    }
}
