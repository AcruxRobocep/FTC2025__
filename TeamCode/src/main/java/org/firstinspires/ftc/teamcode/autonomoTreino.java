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
import org.firstinspires.ftc.teamcode.robo2025.PairMotor;

@Autonomous

public class autonomoTreino extends LinearOpMode {
    DcMotor viperR;
    DcMotor viperL;
    MecanumDrive drive;

    public class upViper implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            /* Programação do viper subir */
            viperR = hardwareMap.get(DcMotor.class,"M6");
            viperL = hardwareMap.get(DcMotor.class,"M5");
            PairMotor viper = new PairMotor();
            viper.setMotors(viperR,viperL);
            /*
            viperL.setTargetPosition(3000); // Tells the motor that the position it should go to is desiredPosition
            viperL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            viperL.setPower(-0.5);
            viperR.setPower(-0.5);

             */
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
            viperR = hardwareMap.get(DcMotor.class,"M6");
            viperL = hardwareMap.get(DcMotor.class,"M5");
            PairMotor viper = new PairMotor();
            viper.setMotors(viperR,viperL);
            viper.sePMtPower(1);
            sleep(1000);
            viper.sePMtPower(0);

            return false;
        }
    }

    public Action downViper(){
        return new downViper();
    }

    public Action upViper(){
        return new upViper();
    }



    public class updatePose implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            drive.localizer.update();
            return false;
        }

    }

    public Action updatePose(){
        return new updatePose();
    }


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d posInicial = new Pose2d(-24.576541312975472, -62.93794794647305, Math.toRadians(0));
        //Pose2d posInicial = new Pose2d(-8.56845649835951,-62.2972301170795,Math.toRadians(180));
         drive = new MecanumDrive(hardwareMap, posInicial);




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
                .strafeToLinearHeading(new Vector2d(10,-44),Math.toRadians(-90));

                //.lineToXLinearHeading(-40,Math.toRadians(90))



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

        waitForStart();


        Actions.runBlocking(
                new SequentialAction(
                        //initBasket.build()

                        initSpecimen.build(),
                        upViper(),
                        downViper(),
                        //updatePose(),
                        positionsamples



                )
        );



    }



}
