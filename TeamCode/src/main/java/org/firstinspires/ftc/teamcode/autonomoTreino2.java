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

public class autonomoTreino2 extends LinearOpMode {
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








        // .strafeTo(new Vector2d(-8.56845649835951,-34));

        TrajectoryActionBuilder traj0 = drive.actionBuilder(new Pose2d(-17.98, -63.75, Math.toRadians(0)))
                .splineTo(new Vector2d(-4.42, -33.89), Math.toRadians(-90))
                .splineTo(new Vector2d(-48.26, -38.00), Math.toRadians(90))
                .splineTo(new Vector2d(-55.72, -55.72), Math.toRadians(45))
                .splineTo(new Vector2d(-59.05, -38.06), Math.toRadians(90))
                .splineTo(new Vector2d(-55.72, -55.91), Math.toRadians(45))
                .splineTo(new Vector2d(-64.15, -36.69), Math.toRadians(109.26))
                .splineTo(new Vector2d(-56.11, -55.91), Math.toRadians(-70.13))
                .splineTo(new Vector2d(-52.19, -16.28), Math.toRadians(62.42))
                .splineTo(new Vector2d(-23.93, -10.99), Math.toRadians(0));


        waitForStart();


        Actions.runBlocking(
                new SequentialAction(
                    traj0.build()


                )
        );



    }



}
