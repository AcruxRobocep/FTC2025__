package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;


@TeleOp
public class robo2025 extends LinearOpMode {
    // Movimentação
    DcMotor RF;
    DcMotor RB;
    DcMotor LF;
    DcMotor LB;

    Servo bracoOUTake;
    Servo bracoINTakeL;
    Servo bracoINTakeR;
    Servo pincaOUTake;
    Servo pulsoOUTake;

    DcMotor rotor;

    DcMotor __inTake;

    DcMotor viperR;
    DcMotor viperL;


    int MAX_VIPER = 3000;
    int MIN_VIPER = 0;

    int time_sleep = 200;

    boolean status = true;

    boolean statusPincaOUTAKE = false;

    boolean statusPulsoOUTAKE = false;
    boolean statusBracoINTAKE = false;
    boolean statusBracoOUTAKE = false;

    ServoController servoControlleR;

    @Override
    public void runOpMode() throws InterruptedException {
        RF = hardwareMap.get(DcMotor.class, "M4");
        RB = hardwareMap.get(DcMotor.class, "M3");
        LF = hardwareMap.get(DcMotor.class, "M2");
        LB = hardwareMap.get(DcMotor.class, "M1");
        rotor = hardwareMap.get(DcMotor.class, "M8");

        bracoOUTake = hardwareMap.get(Servo.class, "S1");
        bracoINTakeL = hardwareMap.get(Servo.class, "S2");
        bracoINTakeR = hardwareMap.get(Servo.class, "S3");
        pincaOUTake = hardwareMap.get(Servo.class, "S4");

        pulsoOUTake = hardwareMap.get(Servo.class, "S5");

        viperR = hardwareMap.get(DcMotor.class, "M6");
        viperL = hardwareMap.get(DcMotor.class, "M5");

        __inTake = hardwareMap.get(DcMotor.class, "M7");


        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();


        while (opModeIsActive()) {

            controle2();
            controle1();


            telemetry.addData("MR", viperR.getCurrentPosition());
            telemetry.addData("ML", viperL.getCurrentPosition());
            telemetry.update();


        }

    }

    public void controle1() {
        movimentacao();
        extensor();
    }

    public void controle2() {
        PairMotor viper = new PairMotor();
        viper.setMotors(viperR, viperL);
        intake();
        outtake();
        viperComandos(viper);
        braco_complexo();

    }

    public void viperComandos(PairMotor v) {
        if (gamepad2.right_trigger > 0) {
            v.sePMtPower(-gamepad2.right_trigger);

        } else if (gamepad2.left_trigger > 0) {
            v.sePMtPower(gamepad2.left_trigger);

        } else {
            v.sePMtPower(0);
        }

    }


    public void movimentacao() {

        double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad1.left_stick_x;
        double r = gamepad1.right_stick_x;
        LF.setPower(y + x + r);
        LB.setPower(y - x + r);
        RF.setPower(y - x - r);
        RB.setPower(y + x - r);

    }


    public static class PairMotor {
        DcMotor m1, m2;

        public void setMotors(DcMotor motor1, DcMotor motor2) {
            m1 = motor1;
            m2 = motor2;
        }

        public void sePMtPower(double power) {
            m1.setPower(power);
            m2.setPower(power);
        }

        public void sePMtPower(double power1, double power2) {
            m1.setPower(power1);
            m2.setPower(power2);
        }


    }

    public void outtake() {

        // Pinça

        if (gamepad2.b) {
            statusPincaOUTAKE = !statusPincaOUTAKE;
            sleep(time_sleep);
        }

        if (statusPincaOUTAKE) {
            pincaOUTake.setPosition(-1);
        } else {
            pincaOUTake.setPosition(1);

        }


        // Pulso
        if (gamepad2.a) {
            statusPulsoOUTAKE = !statusPulsoOUTAKE;
            sleep(time_sleep);
        }


        if (statusPulsoOUTAKE) {
            pulsoOUTake.setPosition(1);
        } else {
            pulsoOUTake.setPosition(0);
        }



        //  Braço
        if(gamepad2.x){
            statusBracoOUTAKE = !statusBracoOUTAKE;
            sleep(time_sleep);
        }

        if(statusBracoOUTAKE){
            bracoOUTake.setPosition(-1);
        }else {
            bracoOUTake.setPosition(1);
        }

    }

    public void intake() {
        // Rotor
        if (gamepad2.right_bumper) {
            rotor.setPower(1);
        } else if (gamepad2.left_bumper) {
            rotor.setPower(-0.7);
        } else {
            rotor.setPower(0);
        }


        // Braço
        if (gamepad2.y) {
            statusBracoINTAKE = !statusBracoINTAKE;
            sleep(300);
        }

        if (statusBracoINTAKE) {
            bracoINTakeL.setPosition(-0.8);
            bracoINTakeR.setPosition(0.8);

        } else {
            bracoINTakeL.setPosition(0.5);
            bracoINTakeR.setPosition(-0.5);
        }


    }

    public void extensor() {
        if (gamepad1.right_stick_button) {
            status = !status;
            sleep(200);
        }

        if (status) {
            __inTake.setPower(-0.7);

        } else if (gamepad1.right_bumper) {
            __inTake.setPower(0.5);
        } else if (gamepad1.left_bumper) {
            __inTake.setPower(-0.5);
        } else {
            __inTake.setPower(0);
        }
    }

    public void braco_complexo() {
        //braco complexo mds

        /*
        if (gamepad2.x) {
            statusBracoOUTAKE = !statusBracoOUTAKE;
            sleep(time_sleep);
        }


        if (statusBracoOUTAKE) {
            bracoOUTake.setPosition(0);
            pulsoOUTake.setPosition(1);
            bracoOUTake.setPosition(-1);
        } else {
            bracoOUTake.setPosition(0);
            pulsoOUTake.setPosition(0);
            bracoOUTake.setPosition(1);
        }
    }
    */
        /*

        if (statusBracoOUTAKE) {
            bracoOUTake.setPosition(-1);
        } else {
            bracoOUTake.setPosition(1);
        }
        /*/

    }
}
