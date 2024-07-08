/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive.writtenCode;

import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TransferController.initPosition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.AvionController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.CataratController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.CollectForbarController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ExtenderController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ForbarOuttakeController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.LiftMotorController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ParbrizController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TurretController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.RotateClawController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ScoringController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.SigurantaOuttakeController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TransferController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TubuleteController;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="extRetard", group="Linear OpMode")
public class extRetard extends LinearOpMode {
    // Putine disclaimere :

    // Ce este loop time-ul?
    // Daca va uitati avem un while in runOpMode cu while(opModeIsActive())
    // Cu cat acest while se evalueaza mai repede cu atat robotul se va misca mai bine pentru ca
    // partile din hard vor fi updatate mai repede.

    // Comentariile in verde de deasupra functiilor se numesc javadov
    // Daca nu stiti ce face o functie puteti da hover peste ea si o sa va apara exact ce scrie in
    // comentariile verzi, try it out!

    /**
     * Pune running mode-ul la motoare
     * Recomand sa va uitati pe ce runningMode-uri sunt disponibile.
     * Pentru sasiu de obicei e bine sa folositi RUN_WITHOUT_ENCODER
     * @param leftFront - motorul din stanga fata
     * @param rightFront - motorul din dreapta fata
     * @param leftBack - motorul din stanga spate
     * @param rightBack - motorul din dreapta spate
     * @param runningMode - ce RunMode vor avea motoarele
     */
    public void setMotorRunningMode(DcMotor leftFront , DcMotor leftBack, DcMotor rightFront,
                                    DcMotor rightBack, DcMotor.RunMode runningMode)
    {
        leftFront.setMode(runningMode);
        rightFront.setMode(runningMode);
        leftBack.setMode(runningMode);
        rightBack.setMode(runningMode);
    }
    /**
     * Pune zeroPowerBehaviour la motoare
     * De obicei se foloseste BREAK
     * @param leftFront - motorul din stanga fata
     * @param rightFront - motorul din dreapta fata
     * @param leftBack - motorul din stanga spate
     * @param rightBack - motorul din dreapta spate
     * @param zeroPowerBehavior - ce RunMode vor avea motoarele
     */
    public void setMotorZeroPowerBehaviour(DcMotor leftFront,DcMotor leftBack, DcMotor rightFront,
                                           DcMotor rightBack, DcMotor.ZeroPowerBehavior zeroPowerBehavior)
    {
        leftFront.setZeroPowerBehavior(zeroPowerBehavior);
        rightFront.setZeroPowerBehavior(zeroPowerBehavior);
        leftBack.setZeroPowerBehavior(zeroPowerBehavior);
        rightBack.setZeroPowerBehavior(zeroPowerBehavior);
    }

    /**
     * Aici punem puterea in motoare ca sa se miste robot centric
     * Pentru mai multe detalii la cum functioneaza citit gm0.
     * @param leftFront - Motorul stanga fata
     * @param leftBack - Motorul stanga spate
     * @param rightFront - Motorul dreapta fata
     * @param rightBack - Motorul dreapta spate
     */
    public void robotCentricDrive(DcMotor leftFront , DcMotor leftBack,
                                  DcMotor rightFront ,DcMotor rightBack,
                                  double leftTrigger, double rightTrigger)
    {
        /// O sa va intrebati cum putem accesa gamepad1 si gamepad2 ?
        /// Probabil sunt variabile globale , n-ar trebui sa va faceti multe griji

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = (-gamepad1.left_trigger + gamepad1.right_trigger)* 1.05; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftBackPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightBackPower = (y + x - rx) / denominator;


//        leftFront.setPower(leftFrontPower);
//        leftBack.setPower(leftBackPower);
//        rightFront.setPower(rightFrontPower);
//        rightBack.setPower(rightBackPower);
    }
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        RobotMap robot = new RobotMap(hardwareMap);

        CataratController cataratController = new CataratController(robot);
        AvionController avionController = new AvionController(robot);
        IntakeController intakeController = new IntakeController(robot);
        CollectForbarController collectForbarController = new CollectForbarController(robot);
        TubuleteController tubuleteController = new TubuleteController(robot);
        TurretController turretController = new TurretController(robot);
        ParbrizController parbrizController = new ParbrizController(robot);
        SigurantaOuttakeController sigurantaOuttakeController = new SigurantaOuttakeController(robot);
        ForbarOuttakeController forbarOuttakeController = new ForbarOuttakeController(robot);
        RotateClawController rotateClawController = new RotateClawController(robot);
        ExtenderController extenderController = new ExtenderController(robot);
        LiftMotorController liftMotorController = new LiftMotorController(forbarOuttakeController,extenderController,robot,turretController);
        robot.forbarCutieIntake.setPosition(initPosition);


        TransferController transferController = new TransferController(
                intakeController,tubuleteController,sigurantaOuttakeController, extenderController,robot);
        ScoringController scoringController = new ScoringController(turretController, sigurantaOuttakeController, parbrizController, rotateClawController);

        cataratController.update();
        avionController.update();
        extenderController.update(ExtenderController.extenderInit);
        rotateClawController.update();
        forbarOuttakeController.update();
        liftMotorController.update();
        intakeController.update();
        collectForbarController.update();
        tubuleteController.update();
        transferController.update();
        turretController.update();
        parbrizController.update();
        sigurantaOuttakeController.update();
        scoringController.update();

        // Declaram motoarele din drive aici
        // Numele motoarelor sunt destul de evidente
        // Ce este intre ghilimele trebuie sa fie acelasi nume ca in config
        DcMotor rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        DcMotor leftFront = hardwareMap.get(DcMotor.class,"leftFront");
        DcMotor rightBack = hardwareMap.get(DcMotor.class,"rightBack");
        DcMotor leftBack = hardwareMap.get(DcMotor.class,"leftBack");

        // Dam toate cele patru motoare si le punem sa mearga fara encodere
        // deoarece asa se misca mai bine
        // Puteti citi documentatia si comentarii pe reddit dar long story short
        // daca merge cu encodere
        // Programul le va folosi pentru a calcula ce viteza sa le dea si asta nu
        // prea e bine pentru loop time.
        setMotorRunningMode(leftFront,leftBack,rightFront,rightBack,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /// Foarte important pentru sasiul de mecanum.
        /// Cititi pe gm0 de ce este necesar
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        /// Aici facem ca motoarele cand nu primesc nicio comanda sa-si tina
        /// pozitia pe care o aveau inainte. Asta va permite sa nu fiti impinsi foarte usor
        /// de alti roboti
        setMotorZeroPowerBehaviour(leftFront,leftBack,rightFront,rightBack,
                DcMotor.ZeroPowerBehavior.BRAKE);

        /// Este clar ca currentGamepad1 este de la primul driver
        /// currentGamepad2 este de la al doilea driver
        /// De ce avem nevoie de previousGamepad1 si previousGamepad2?
        /// Raspunsul scurt este pentru ca avem nevoie sa vedem cand am apasat o data pe buton
        /// Pentru mai multe explicatii uitati-va pe gm0.

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        boolean usesBeam = true;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        collectForbarController.currentStatus = CollectForbarController.CollectStatus.PLAY;
        collectForbarController.update();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (isStopRequested()) return;

            int liftCurrentPosition = robot.liftMotor.getCurrentPosition();
            int extenderCurrentPosition = robot.rightExtension.getCurrentPosition();

            /// Updatam motoarele cu puterile necesare ca sa miscam sasiul
            /// Vei vedea ca folosim aceeasi chestie ca pe gm0
            /// Doar am pus-o in alta functie pentru ca nu prea o sa o modificam des
            robotCentricDrive(leftFront,leftBack,rightFront,rightBack,gamepad1.left_trigger,gamepad1.right_trigger);



            /// Destul de clar ce fac aceste linii
            /// Uitati-va pe gm0 pentru explicatii
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);


            if(liftCurrentPosition>=-120 && liftMotorController.currentStatus == LiftMotorController.LiftStatus.GOING_DOWN)
            {
                liftMotorController.currentStatus = LiftMotorController.LiftStatus.INIT;
            }
            /// Control lift manual
            if (gamepad2.left_stick_y >0)
            {
                /// Ii dau clip la currentPosition intre initPosition si highPosition.
                liftMotorController.currentPosition = liftMotorController.currentPosition+10;
                extenderController.currentStatus = ExtenderController.ExtenderStatus.FIX;
            }
            else
            if (gamepad2.left_stick_y <0)
            {
                /// Ii dau clip la currentPosition intre initPosition si highPosition.
                liftMotorController.currentPosition = liftMotorController.currentPosition-10;
                extenderController.currentStatus = ExtenderController.ExtenderStatus.FIX;
            }
            if (gamepad2.right_stick_y >0)
            {
                /// Ii dau clip la currentPosition intre initPosition si highPosition.
                extenderController.CurrentPosition = extenderController.CurrentPosition-5;
            }
            if (gamepad2.right_stick_y <0)
            {
                /// Ii dau clip la currentPosition intre initPosition si highPosition.
                extenderController.CurrentPosition = extenderController.CurrentPosition+5;
            }
            /// Lift going to mosaic

            cataratController.update();
            avionController.update();
            extenderController.update(extenderCurrentPosition);
            rotateClawController.update();
            forbarOuttakeController.update();
            liftMotorController.update();
            intakeController.update();
            collectForbarController.update();
            tubuleteController.update();
            transferController.update();
            turretController.update();
            parbrizController.update();
            sigurantaOuttakeController.update();
            scoringController.update();
        }
    }
}