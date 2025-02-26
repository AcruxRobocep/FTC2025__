package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robo2025.PairMotor;

import java.lang.reflect.Array;
import java.util.Arrays;

@Autonomous

public class autonomoTreino2 extends LinearOpMode {
    DcMotor viperR;
    DcMotor viperL;

    double velRotor = 0.5;
    int sleepRunRotor = 1000;
    DcMotor rotor;
    MecanumDrive drive;


    Servo  bracoOUTake  = hardwareMap.get(Servo.class, "S1");
    Servo bracoINTakeL  = hardwareMap.get(Servo.class ,"S2");
    Servo bracoINTakeR  = hardwareMap.get(Servo.class ,"S3");
    Servo pincaOUTake  = hardwareMap.get(Servo.class ,"S5");
    Servo pulsoOUTake = hardwareMap.get(Servo.class,"S4");


    public class upViper implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            /* Programação do viper subir */
            viperR = hardwareMap.get(DcMotor.class, "M6");
            viperL = hardwareMap.get(DcMotor.class, "M5");
            PairMotor viper = new PairMotor();
            viper.setMotors(viperR, viperL);

            viper.sePMtPower(-0.5);
            sleep(2000);
            viper.sePMtPower(0);

            return false;
        }
    }

    public class downViper implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            /* Programação do viper subir */
            viperR = hardwareMap.get(DcMotor.class, "M6");
            viperL = hardwareMap.get(DcMotor.class, "M5");
            PairMotor viper = new PairMotor();
            viper.setMotors(viperR, viperL);
            viper.sePMtPower(1);
            sleep(1000);
            viper.sePMtPower(0);

            return false;
        }
    }

    public Action downViper() {
        return new downViper();
    }

    public Action upViper() {
        return new upViper();
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

    public class rotorClock implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            rotor = hardwareMap.get(DcMotor.class, "M8");

            rotor.setPower(-velRotor);
            sleep(sleepRunRotor);
            rotor.setPower(0);
            return false;
        }
    }

    public Action rotorClock() {
        return new rotorClock();
    }


    public class rotorUNClock implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            rotor = hardwareMap.get(DcMotor.class, "M8");
            rotor.setPower(velRotor);
            sleep(sleepRunRotor);
            rotor.setPower(0);
            return false;
        }


    }

    public Action rotorUNClock() {
        return new rotorClock();
    }


    public class intakeOutake implements  Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            bracoINTakeL.setPosition(-0.8);
            bracoINTakeR.setPosition(0.8);
            sleep(500);
            rotorClock();
            sleep(1000);
            bracoINTakeL.setPosition(-0.1);
            bracoINTakeR.setPosition(0.1);
            sleep(500);

            pincaOUTake.setPosition(-1);

            bracoOUTake.setPosition(1);

            pincaOUTake.setPosition(1);







            return false;
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d posInicial = new Pose2d(-24.576541312975472, -62.93794794647305, Math.toRadians(0));
        drive = new MecanumDrive(hardwareMap, posInicial);



        TrajectoryActionBuilder traj0 = drive.actionBuilder(new Pose2d(-17.98, -63.75, Math.toRadians(0)))
                // Trajetória basquete+espécimes
                .splineTo(new Vector2d(-4.42, -33.89), Math.toRadians(-90))
                .splineTo(new Vector2d(-48.26, -38.00), Math.toRadians(90))
                .splineTo(new Vector2d(-55.72, -55.72), Math.toRadians(45))
                .splineTo(new Vector2d(-59.05, -38.06), Math.toRadians(90))
                .splineTo(new Vector2d(-55.72, -55.91), Math.toRadians(45))
                .splineTo(new Vector2d(-64.15, -36.69), Math.toRadians(109.26))
                .splineTo(new Vector2d(-56.11, -55.91), Math.toRadians(-70.13))
                .splineTo(new Vector2d(-52.19, -16.28), Math.toRadians(62.42))
                .splineTo(new Vector2d(-23.93, -10.99), Math.toRadians(0));


        TrajectoryActionBuilder traj1 = drive.
                actionBuilder(new Pose2d(24.58, -62.94, Math.toRadians(0)))
                //Trajetória espécimes
                .strafeToLinearHeading(new Vector2d(12.00, -34.00), Math.toRadians(90.00))
                .strafeTo(new Vector2d(40.18, -33.78))
                .splineTo(new Vector2d(36.67, -2.79), Math.toRadians(0))
                .splineTo(new Vector2d(49.48, -20.56), Math.toRadians(265.24))
                .strafeTo(new Vector2d(49.00, -62.00))
                .strafeTo(new Vector2d(49.00, -9.00))
                .strafeTo(new Vector2d(59.00, -9.00))
                .strafeTo(new Vector2d(59.00, -62.00))
                .strafeTo(new Vector2d(59.00, -9.00))
                .strafeTo(new Vector2d(64.98, -8.99))
                .strafeTo(new Vector2d(64.98, -62.00))
                .strafeTo(new Vector2d(49.00, -39.00))
                .strafeTo(new Vector2d(49.00, -62.00))
                .strafeTo(new Vector2d(12.00, -34.00))
                .strafeTo(new Vector2d(49.00, -39.00))
                .strafeTo(new Vector2d(49.00, -62.00))
                .strafeTo(new Vector2d(12.00, -34.00))
                .strafeTo(new Vector2d(49.00, -39.00))
                .strafeTo(new Vector2d(12.00, -34.00))
                .strafeTo(new Vector2d(54.00, -63.00));

        waitForStart();

        /*
        Actions.runBlocking(
                new SequentialAction(
                    blockExtensor()
                        ,rotorClock()
                    //traj0.build()
                    //,traj1.build()

                )
        );
        */


        Actions.runBlocking(blockExtensor());
        Actions.runBlocking(
                new SequentialAction(
                        //rotorClock(),
                        blockExtensor()
                        ,traj0.build()
                )
        );


    }
}
