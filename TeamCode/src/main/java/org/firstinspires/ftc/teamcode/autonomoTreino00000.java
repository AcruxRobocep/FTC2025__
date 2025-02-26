package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.robo2025.PairMotor;

@Autonomous

public class autonomoTreino00000 extends LinearOpMode {


    MecanumDrive drive;


    Servo bracoOUTake;
    Servo bracoINTakeL;
    Servo bracoINTakeR;
    Servo pincaOUTake;
    Servo pulsoOUTake;

    DcMotor rotor;

    DcMotor __inTake;

    DcMotor viperR;
    DcMotor viperL;

    int MAX_VIPER = 1000;
    public class upViper implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            /* Programação do viper subir */

            PairMotor viper = new PairMotor();
            viper.setMotors(viperR,viperL);
            viper.sePMtPower(-0.5);
            sleep(2400);
            viper.sePMtPower(0);



            /*
            viperL.setTargetPosition(MAX_VIPER);
            viperR.setTargetPosition(MAX_VIPER);
            viper.sePMtPower(0.5);
            viperR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            viperL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            */


            return false;
        }
    }

    public class downViper implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            /* Programação do viper subir */
            /*
            PairMotor viper = new PairMotor();
            viperL.setTargetPosition(-3000);
            viperR.setTargetPosition(-3000);
            viper.sePMtPower(0.3);
            viperL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            viperR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);
            pincaOUTake.setPosition(1);
            */
            return false;
        }
    }

    public class blockExtensor implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            DcMotor extensor = hardwareMap.get(DcMotor.class, "M7");
            extensor.setPower(-0.5);
            sleep(10);


            return false;
        }
    }

    public Action blockExtensor() {
        return new blockExtensor();
    }



    public Action downViper(){
        return new downViper();
    }

    public Action upViper(){
        return new upViper();
    }



    @Override
    public void runOpMode() throws InterruptedException {
        //Pose2d posInicial = new Pose2d(-24.576541312975472, -62.93794794647305, Math.toRadians(0));
        Pose2d posInicial = new Pose2d(24.58,-62.24,Math.toRadians(180));
        drive = new MecanumDrive(hardwareMap, posInicial);

        rotor = hardwareMap.get(DcMotor.class, "M8");

        bracoOUTake  = hardwareMap.get(Servo.class, "S1");
        bracoINTakeL  = hardwareMap.get(Servo.class ,"S2");
        bracoINTakeR  = hardwareMap.get(Servo.class ,"S3");
        pincaOUTake  = hardwareMap.get(Servo.class ,"S4");
        //pulsoOUTake = hardwareMap.get(Servo.class,"S5");

        viperR = hardwareMap.get(DcMotor.class,"M6");
        viperL = hardwareMap.get(DcMotor.class,"M5");

        __inTake = hardwareMap.get(DcMotor.class,"M7");
        /*
        viperR.setDirection(DcMotorSimple.Direction.REVERSE);

        viperL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        viperR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */
        TrajectoryActionBuilder initSpecimenZoneOb = drive.actionBuilder(new Pose2d(24.58, -62.94, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(10,-42),Math.toRadians(-90));

        TrajectoryActionBuilder initBasket = drive.actionBuilder(posInicial)
                //alinhar ao basket
                .strafeTo(new Vector2d(-60,-39))
                .turn(Math.toRadians(45))
                .waitSeconds(1)
                .turn(Math.toRadians(45))
                .waitSeconds(1)
                // Posicao pegar sample
                .setTangent(90)
                .lineToY(-45)

                ;


        TrajectoryActionBuilder initSpecimen = drive.actionBuilder(posInicial)
                //Alinhar no high chamber
                .strafeToLinearHeading(new Vector2d(10,-42),Math.toRadians(-90));

        //.lineToXLinearHeading(-40,Math.toRadians(90))

        Action prenderespecime = initSpecimen.endTrajectory().fresh()
                .strafeTo(new Vector2d(10,-39))
                .build();
        Action prenderespecime2 = initSpecimenZoneOb.endTrajectory().fresh()
                .strafeTo(new Vector2d(10,-39))
                .build();

        // .strafeTo(new Vector2d(-8.56845649835951,-34));

        Action positionsamples = initSpecimen.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-60,-45),Math.toRadians(90))
                .waitSeconds(0.5)
                .turn(Math.toRadians(-45))
                .build();
        /*
        TrajectoryActionBuilder trajectory0 = drive.actionBuilder(new Pose2d(-17.33,-63.47,Math.toRadians(0)))
                .splineTo(new Vector2d(-3.57, -41.70), Math.toRadians(270))
                .waitSeconds(0.5)
                .splineTo(new Vector2d(-48.67, -38.74), Math.toRadians(90.68))
                .waitSeconds(0.5)
                .splineTo(new Vector2d(-53.54, -53.54), Math.toRadians(56.06))
                .waitSeconds(0.5)
                .splineTo(new Vector2d(-59.11, -35.61), Math.toRadians(89.35))
                .waitSeconds(0.5)
                .splineTo(new Vector2d(-53.54, -53.54), Math.toRadians(241.93))
                .waitSeconds(0.5)
                .splineTo(new Vector2d(-59.81, -30.38), Math.toRadians(165.17));
*/
        bracoOUTake.setPosition(0.2);
        pincaOUTake.setPosition(-1);


        waitForStart();


        Actions.runBlocking(
                new SequentialAction(
                        //initBasket.build()
                        blockExtensor(),
                        initSpecimenZoneOb.build(),

                        upViper(),
                        prenderespecime2

                        /*blockExtensor(),
                        upViper()
                        //downViper()

                         */
                        /*
                        //updatePose(),
                        positionsamples
                        */


                )
        );



    }



}