package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;
@Config
public class TubuleteController {
    public enum CollectStatus{
        COLECTARE,
        TRANSFER,
        BLOCARE
    }

    public CollectStatus currentStatus = CollectStatus.COLECTARE;
    public CollectStatus previousStatus = null;

    public static double collectPosition1 = 0.91; //0.13, 0.91 pt paralel
    public double collectPosition2 = 1-collectPosition1;

    public static double transferPosition1 = 1;
    public double transferPosition2 = 1-transferPosition1;

    public double blocarePosition1 = 0.05;
    public double blocarePosition2 = 1- blocarePosition1;

    private Servo TransferServo = null;

    public TubuleteController(RobotMap robot)
    {
        TransferServo = robot.TransferServo;
    }

    public void update()
    {
        if (currentStatus != previousStatus)
        {
            previousStatus = currentStatus;
            switch (currentStatus)
            {
                case COLECTARE:
                {
                    TransferServo.setPosition(collectPosition1);
                    break;
                }
                case TRANSFER:
                {
                    TransferServo.setPosition(transferPosition1);
                    break;
                }
                case BLOCARE:
                {
                    TransferServo.setPosition(blocarePosition1);
                }
            }
        }
    }
}
