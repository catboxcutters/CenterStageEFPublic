package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class CataratController {
    public enum CataratStatus{
        STOP,

        UP,
        DOWN,
    }
    public CataratStatus currentStatus = CataratController.CataratStatus.STOP;
    public CataratController.CataratStatus previousStatus = null;

    private CRServo  hangingRight= null;
    private CRServo  hangingLeft= null;

    public CataratController(RobotMap robot)
    {
        hangingLeft = robot.hangingLeft;
        hangingRight = robot.hangingRight;
    }
    public void update()
    {
        if (currentStatus != previousStatus)
        {
            previousStatus = currentStatus;
            switch (currentStatus)
            {
                case STOP:
                {
                    hangingRight.setPower(0);
                    hangingLeft.setPower(0);
                    break;
                }
                case UP:
                {
                    hangingRight.setPower(0.9);
                    hangingLeft.setPower(-0.9);
                    break;
                }
                case DOWN:
                {
                    hangingRight.setPower(-0.9);
                    hangingLeft.setPower(0.9);
                    break;
                }
            }
            }
}}
