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
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.AvionController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.CataratController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.CollectForbarController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ExtenderController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ForbarOuttakeController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.LiftMotorController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.OuttakeSlidesController;
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

@TeleOp(name="TeleOpCode", group="Linear OpMode")
public class TeleOpCode extends LinearOpMode {
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


        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }
    ElapsedTime GlobalTimer = new ElapsedTime();
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
        OuttakeSlidesController outtakeSlidesController = new OuttakeSlidesController(robot);
        robot.forbarCutieIntake.setPosition(initPosition);
        robot.clawRotate.setPosition(RotateClawController.verticalServoPosition);

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
        outtakeSlidesController.update();
        robot.imu.update();

//        imu = hardwareMap.get(IMU.class, "imu");
        // Declaram motoarele din drive aici
        // Numele motoarelor sunt destul de evidente
        // Ce este intre ghilimele trebuie sa fie acelasi nume ca in config
        DcMotor rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        DcMotor leftFront = hardwareMap.get(DcMotor.class,"leftFront");
        DcMotor rightBack = hardwareMap.get(DcMotor.class,"rightBack");
        DcMotor leftBack = hardwareMap.get(DcMotor.class,"leftBack");
        MotorConfigurationType mct1, mct2, mct3, mct4;
        mct1 = rightBack.getMotorType().clone();
        mct1.setAchieveableMaxRPMFraction(1.0);
        rightBack.setMotorType(mct1);

        mct2 = rightFront.getMotorType().clone();
        mct2.setAchieveableMaxRPMFraction(1.0);
        rightFront.setMotorType(mct2);

        mct3 = leftFront.getMotorType().clone();
        mct3.setAchieveableMaxRPMFraction(1.0);
        leftFront.setMotorType(mct3);

        mct4 = leftBack.getMotorType().clone();
        mct4.setAchieveableMaxRPMFraction(1.0);
        leftBack.setMotorType(mct4);

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
        double differentThanFalseFalse =0;
        GlobalTimer.reset();
        collectForbarController.currentStatus = CollectForbarController.CollectStatus.PLAY;
        collectForbarController.update();
        outtakeSlidesController.currentStatus= OuttakeSlidesController.ExtensionStatus.COMPACT;
        outtakeSlidesController.update();
        boolean imuReset = false;
        int ok=-1;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (isStopRequested()) return;

            int liftCurrentPosition = robot.liftMotor.getCurrentPosition();
            int extenderCurrentPosition = robot.leftExtension.getCurrentPosition();

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

            /// Am apasat dpad_left ->
            // 4Bar ul intake se duce in pozitie
            // Colectare incepe.
            // Switch/Toggle - Continous press
            // Dpad_left- > 4Bar-ul se duce , incepe colectare , Dpad_left - > 4barul se intoarce (Switch)
            // Cat timp e apasat se intampla ce s-a scris mai sus (Continous)

            if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left)
            {
                if (intakeController.currentStatus == IntakeController.IntakeStatus.STOP)
                {
                    tubuleteController.currentStatus = TubuleteController.CollectStatus.COLECTARE;
                    intakeController.currentStatus = IntakeController.IntakeStatus.COLLECT_DRIVE;
                    if (collectForbarController.currentStatus == CollectForbarController.CollectStatus.PLAY)
                    {
                        collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_DRIVE;
                    }
                }
                else
                {
//                    tubuleteController.currentStatus = TubuleteController.CollectStatus.BLOCARE;
                    intakeController.currentStatus = IntakeController.IntakeStatus.STOP;
                }
            }
            if(currentGamepad2.left_bumper && previousGamepad2.left_bumper)
            {
                if(collectForbarController.currentStatus != CollectForbarController.CollectStatus.PLAY) {
                    collectForbarController.currentStatus = CollectForbarController.CollectStatus.PLAY;
                }
                }

            if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right)
            {
//                transferController.currentStatus= TransferController.TransferStatus.PLACE_BOX_IN_COLLECT;
                intakeController.currentStatus = IntakeController.IntakeStatus.REVERSE;
                collectForbarController.currentStatus=CollectForbarController.CollectStatus.COLLECT_DRIVE_STACK;
            }

            /// Apas pe right_trigger
            /// Se blocheaza + Reverse colectare
            /// Flip , dupa flip opresc colectarea
            /// Asteapta flip ul sa se termine
            /// Transfer pixelii dintr-o cutie in alta si apoi inchid siguranta
            /// Cutia de la intake revine la pozitia de colectare

            //if (currentGamepad2.right_trigger>0 && previousGamepad2.right_trigger==0)
            if(currentGamepad2.b && !previousGamepad2.b)
            {
                if (transferController.currentStatus == TransferController.TransferStatus.INIT)
//                        && extenderCurrentPosition <= 0)
                {
                    transferController.actualTimeForExtendo = 0;
                    // Cand fac transferul vreau sa fie inchis parbrizul
                    parbrizController.currentStatus = ParbrizController.ParbrizStatus.CLOSED;
                    // Sa fie deschis pixel2 ca sa nu impiedice colectarea
                    transferController.currentStatus = TransferController.TransferStatus.BLOCHEAZA_TUBULETE;
                }
            }
            /// Lift going to low
            if (currentGamepad2.a && !previousGamepad2.a)
            {
                if (liftMotorController.currentStatus == LiftMotorController.LiftStatus.INIT)
                {
                    liftMotorController.currentStatus = LiftMotorController.LiftStatus.LOW;
                    outtakeSlidesController.currentStatus = OuttakeSlidesController.ExtensionStatus.FAR;
                }
                else
                {
                    if (rotateClawController.currentStatus == RotateClawController.RotateStatus.VERTICAL)
                    {
                        liftMotorController.currentStatus = LiftMotorController.LiftStatus.INIT;
                        outtakeSlidesController.currentStatus = OuttakeSlidesController.ExtensionStatus.COMPACT;
                        parbrizController.currentStatus = ParbrizController.ParbrizStatus.CLOSED;
                    }
                    else if(rotateClawController.currentStatus== RotateClawController.RotateStatus.RUNTO && ok==0)
                    {
                        liftMotorController.currentStatus = LiftMotorController.LiftStatus.INIT;
                        outtakeSlidesController.currentStatus = OuttakeSlidesController.ExtensionStatus.COMPACT;
                        parbrizController.currentStatus = ParbrizController.ParbrizStatus.CLOSED;
                        rotateClawController.currentStatus = RotateClawController.RotateStatus.VERTICAL;
                    }
                    else if(rotateClawController.currentStatus == RotateClawController.RotateStatus.HORIZONTAL || (ok==1 && rotateClawController.currentStatus== RotateClawController.RotateStatus.RUNTO))
                    {
                        rotateClawController.currentStatus = RotateClawController.RotateStatus.VERTICAL;
                    }
                }
            }
            /// Control lift manual
            if (gamepad2.left_stick_y >0)
            {
                /// Ii dau clip la currentPosition intre initPosition si highPosition.
//                liftMotorController.currentPosition =
//                        Math.max(liftMotorController.highPosition,Math.min(liftMotorController.currentPosition-10,//era +10, dar mergea invers
//                                liftMotorController.retardPosition));
                liftMotorController.currentPosition =liftMotorController.currentPosition+15;
              //  extenderController.currentStatus = ExtenderController.ExtenderStatus.FIX;
            }
            else
            if (gamepad2.left_stick_y <0)
            {
                /// Ii dau clip la currentPosition intre initPosition si highPosition.
//                liftMotorController.currentPosition =
//                        Math.max(liftMotorController.highPosition,Math.min(liftMotorController.currentPosition+15,//era -10, dar mergea invers
//                                liftMotorController.retardPosition));
                liftMotorController.currentPosition =liftMotorController.currentPosition-15;
               // extenderController.currentStatus = ExtenderController.ExtenderStatus.FIX;
            }


            /// Lift going to mosaic
            if (currentGamepad2.x && !previousGamepad2.x)
            {
                if (liftMotorController.currentStatus == LiftMotorController.LiftStatus.INIT)
                {
                    liftMotorController.currentStatus = LiftMotorController.LiftStatus.liftMosaic;
                    outtakeSlidesController.currentStatus = OuttakeSlidesController.ExtensionStatus.FAR;
                    //rotateClawController.currentStatus = RotateClawController.RotateStatus.HORIZONTAL;
                }
                else
                {
                    if (rotateClawController.currentStatus == RotateClawController.RotateStatus.HORIZONTAL)
                    {
                        rotateClawController.currentStatus = RotateClawController.RotateStatus.VERTICAL;
                        parbrizController.currentStatus = ParbrizController.ParbrizStatus.CLOSED;
                        liftMotorController.currentStatus = LiftMotorController.LiftStatus.INIT;
                        outtakeSlidesController.currentStatus = OuttakeSlidesController.ExtensionStatus.COMPACT;
                    }
                    //{
                      //  liftMotorController.currentStatus = LiftMotorController.LiftStatus.GOING_DOWN;
                        //
                   // }
                    //else if(rotateClawController.currentStatus == RotateClawController.RotateStatus.HORIZONTAL)
                    //{
                      //  rotateClawController.currentStatus = RotateClawController.RotateStatus.VERTICAL;
                    //}
                }
            }
            if(liftMotorController.liftMotor.getCurrentPosition() > liftMotorController.mosaicPosition-10 && liftMotorController.currentStatus== LiftMotorController.LiftStatus.liftMosaic)
            {
                rotateClawController.currentStatus = RotateClawController.RotateStatus.HORIZONTAL;
            }
            // Apas pe left_bumper vreau sa-mi pice doar un singur pixel cand e in pozitie verticala
            // Apas pe right_bumper vreau sa-mi pice ambii

            // Eu sunt cu pixelii ambii in cutie doar cu siguranta de la outtake pusa
            // Siguranta de la outtake , pabriz , pixel2 (dreapta)

            // left_bumper -> pixel2 closed + delay + siguranta outtake
            // right_bumper -> siguranta + pixel2 open

            if(currentGamepad1.b && !previousGamepad1.b)
            {
                sigurantaOuttakeController.currentStatus = SigurantaOuttakeController.SigurantaOuttakeStatus.RETARD;
            }
            if(currentGamepad1.a && !previousGamepad1.a)
            {
                if(forbarOuttakeController.currentStatus == ForbarOuttakeController.ForbarStatus.PLACE_ON_BACKBOARD_WITH_ANGLE) {
                    forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.PLACE_ON_BACKBOARD;
                }
                else {
                    forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.PLACE_ON_BACKBOARD_WITH_ANGLE;
                }
            }
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper)
            {
                if (scoringController.currentStatus == ScoringController.ScoringStatus.INIT)
                {
                    scoringController.currentStatus = ScoringController.ScoringStatus.DROP_BOTH_PIXELS;
                }
                if(rotateClawController.currentStatus == RotateClawController.RotateStatus.RUNTO && ok==1){
                    sigurantaOuttakeController.currentStatus = SigurantaOuttakeController.SigurantaOuttakeStatus.OPEN;
                    parbrizController.currentStatus = ParbrizController.ParbrizStatus.OPEN;
                }
            }

            if (currentGamepad2.y && !previousGamepad2.y
                && forbarOuttakeController.currentStatus == ForbarOuttakeController.ForbarStatus.PLACE_ON_BACKBOARD)
            {
                if (rotateClawController.currentStatus == RotateClawController.RotateStatus.HORIZONTAL || (rotateClawController.currentStatus== RotateClawController.RotateStatus.RUNTO && ok==1))
                {
                    rotateClawController.currentStatus = RotateClawController.RotateStatus.VERTICAL;
                }
                else if(rotateClawController.currentStatus == RotateClawController.RotateStatus.VERTICAL || (rotateClawController.currentStatus== RotateClawController.RotateStatus.RUNTO && ok==0))
                {
                    rotateClawController.currentStatus = RotateClawController.RotateStatus.HORIZONTAL;
                }
            }



            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper)
            {
                if (extenderController.currentStatus == ExtenderController.ExtenderStatus.INIT)
                {
                    extenderController.currentStatus = ExtenderController.ExtenderStatus.FAR;
                    tubuleteController.currentStatus = TubuleteController.CollectStatus.COLECTARE;
                    intakeController.currentStatus = IntakeController.IntakeStatus.COLLECT_DRIVE;
                    collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_DRIVE;
                }
                else
                {

                    extenderController.currentStatus = ExtenderController.ExtenderStatus.INIT;

                    // Fac transferul cu extendo

                    // Cand fac transferul vreau sa fie inchis parbrizul
                    parbrizController.currentStatus = ParbrizController.ParbrizStatus.CLOSED;
                    // Sa fie deschis pixel2 ca sa nu impiedice colectarea

                    // Pun timpul pentru extendo
                    transferController.actualTimeForExtendo = TransferController.timerExtendoToInit;

                    transferController.currentStatus = TransferController.TransferStatus.BLOCHEAZA_TUBULETE;
                }
            }
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper)
            {
                if (extenderController.currentStatus == ExtenderController.ExtenderStatus.INIT)
                {
                    extenderController.currentStatus = ExtenderController.ExtenderStatus.CLOSE;
                }
                else
                {
                    extenderController.currentStatus = ExtenderController.ExtenderStatus.INIT;
                }
            }
            if (currentGamepad1.x && !previousGamepad1.x)
            {
                robot.imu.resetImu();
                imuReset=true;
            }
//            if (!(robot.beamFront.getState() == false&& robot.beamBack.getState() == false))
//            {
//                differentThanFalseFalse = GlobalTimer.seconds();
//            }
            /// Daca robotul vede doi pixeli in fata , folosim break beam - uri
            if (usesBeam && robot.beamFront.getState() == false && robot.beamBack.getState() == false)
            {
                if (transferController.currentStatus == TransferController.TransferStatus.INIT)
                {
                    if (extenderController.currentStatus == ExtenderController.ExtenderStatus.INIT)
                    {
                        // Cand fac transferul vreau sa fie inchis parbrizul
                        parbrizController.currentStatus = ParbrizController.ParbrizStatus.CLOSED;
                        // Sa fie deschis pixel2 ca sa nu impiedice colectarea

                        // Nu trebuie sa tin cont de extendo
                        transferController.actualTimeForExtendo = 0;

                        transferController.currentStatus = TransferController.TransferStatus.BLOCHEAZA_TUBULETE;
                    }
                    else
                    {
                        // Fac transferul cu extendo

                        // Cand fac transferul vreau sa fie inchis parbrizul
                        //parbrizController.currentStatus = ParbrizController.ParbrizStatus.CLOSED;
                        //am parbrizul mereu inchis cand liftul e jos

                        // Sa fie deschis pixel2 ca sa nu impiedice colectarea

                        // Pun timpul pentru extendo
                        transferController.actualTimeForExtendo = TransferController.timerExtendoToInit;

                        transferController.currentStatus = TransferController.TransferStatus.BLOCHEAZA_TUBULETE;
                       // extenderController.currentStatus = ExtenderController.ExtenderStatus.INIT;
                    }
                }
            }

            if(currentGamepad1.y && !previousGamepad1.y)
            {
                if (avionController.currentStatus == AvionController.LaunchStatus.INIT)
                {
                    avionController.currentStatus = AvionController.LaunchStatus.LAUNCH;
                }
                else
                {
                    avionController.currentStatus = AvionController.LaunchStatus.INIT;
                }
            }
            if(currentGamepad2.dpad_down==true)
            {
                if(cataratController.currentStatus == CataratController.CataratStatus.STOP)
                {
                    cataratController.currentStatus = CataratController.CataratStatus.DOWN;
                }
            }
            else
            {
                cataratController.currentStatus = CataratController.CataratStatus.STOP;
            }
            if(currentGamepad2.dpad_up==true)
            {
                if(cataratController.currentStatus == CataratController.CataratStatus.STOP)
                {
                    cataratController.currentStatus = CataratController.CataratStatus.UP;
                }
            }

            if(gamepad2.right_trigger != 0)
            {
                collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_DRIVE_STACK;
            }
            else
            {
                if(collectForbarController.currentStatus == CollectForbarController.CollectStatus.COLLECT_DRIVE_STACK)
                {
                    collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_DRIVE;
                }
            }

            if(gamepad2.left_trigger != 0)
            {
                collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_DRIVE_STACK_LOW;
            }
            else
            {
                if(collectForbarController.currentStatus == CollectForbarController.CollectStatus.COLLECT_DRIVE_STACK_LOW)
                {
                    collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_DRIVE;
                }
            }
            if(forbarOuttakeController.currentStatus != ForbarOuttakeController.ForbarStatus.PLACE_ON_BACKBOARD_WITH_ANGLE &&  liftMotorController.currentStatus != LiftMotorController.LiftStatus.INIT && liftMotorController.liftMotor.getCurrentPosition()<-100 && imuReset==true && Math.toDegrees(robot.imu.getHeading())<45 && Math.toDegrees(robot.imu.getHeading())>-45 && outtakeSlidesController.outtakeSlides.getPosition()<0.45)
            {
                turretController.currentStatus= TurretController.TurretStatus.RUNTO;
                turretController.targetposition = turretController.initPosition + (0.003 * (0-Math.toDegrees(robot.imu.getHeading())));
                if(rotateClawController.currentStatus== RotateClawController.RotateStatus.VERTICAL) ok=0;
                else if(rotateClawController.currentStatus== RotateClawController.RotateStatus.HORIZONTAL) ok=1;
                if(ok==0) {
                    rotateClawController.currentStatus = RotateClawController.RotateStatus.RUNTO;
                    rotateClawController.targetposition = rotateClawController.verticalServoPosition - (0.0010 * (0 - Math.toDegrees(robot.imu.getHeading())));
                }
                else if(ok==1)
                {
                    rotateClawController.currentStatus = RotateClawController.RotateStatus.RUNTO;
                    rotateClawController.targetposition = rotateClawController.horizontalServoPosition - (0.0010 * (0 - Math.toDegrees(robot.imu.getHeading())));
                }
                telemetry.addData("using turret", true);
            }
            else if(forbarOuttakeController.currentStatus == ForbarOuttakeController.ForbarStatus.PLACE_ON_BACKBOARD_WITH_ANGLE)
            {
                turretController.currentStatus = TurretController.TurretStatus.INIT;
                rotateClawController.currentStatus = RotateClawController.RotateStatus.VERTICAL;
            }
            else if(liftMotorController.currentStatus != LiftMotorController.LiftStatus.INIT && liftMotorController.liftMotor.getCurrentPosition()<-100 && imuReset==true && outtakeSlidesController.outtakeSlides.getPosition()<0.45)
            {
                if(rotateClawController.currentStatus== RotateClawController.RotateStatus.VERTICAL) ok=0;
                else if(rotateClawController.currentStatus== RotateClawController.RotateStatus.HORIZONTAL) ok=1;
                turretController.currentStatus= TurretController.TurretStatus.RUNTO;
//                rotateClawController.currentStatus = RotateClawController.RotateStatus.RUNTO;
                telemetry.addData("using turret", true);
            }
            else{
                turretController.currentStatus= TurretController.TurretStatus.INIT;
            }

//            if(gamepad1.dpad_right)
//            {
//                extenderCurrentPosition=extenderCurrentPosition+15;
//            }
//            else if(gamepad1.dpad_left)
//            {
//               extenderCurrentPosition=extenderCurrentPosition-15;
//            }
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
            outtakeSlidesController.update();
            robot.imu.update();

            telemetry.addData("Motor extensie stanga", robot.leftExtension.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Motor extensie dreapta", robot.rightExtension.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Motor extensie outtake", robot.liftMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Motor intake", robot.intakeMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Servo forbar Intake",robot.forbarIntake.getPosition());
            telemetry.addData("Beam Back",robot.beamBack.getState());
            telemetry.addData("Beam Front",robot.beamFront.getState());
            telemetry.addData("Right Trigger" , gamepad2.right_trigger);
            telemetry.addData("Left Trigger", gamepad2.left_trigger);
            telemetry.addData("Collect Forbar status", collectForbarController.currentStatus);
            telemetry.addData("Transfer Encoder", robot.encoderForbarCutie.getVoltage());
            telemetry.addData("rotateclaw status", rotateClawController.currentStatus);
            telemetry.addData("timerlift", liftMotorController.timertoinit.seconds());
            telemetry.addData("HEADING", Math.toDegrees(robot.imu.getHeading()));
            telemetry.update();
        }
    }
}