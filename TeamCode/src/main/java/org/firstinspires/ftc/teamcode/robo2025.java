package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class robo2025 extends LinearOpMode {
    // Movimentação
    DcMotor RF;
    DcMotor RB;
    DcMotor LF;
    DcMotor LB;

    Servo ServoOuttake;
    Servo bracoOUTake;
    Servo ServoPulsoIntake;
    Servo pincaOUTake;

    DcMotor rotor;

    DcMotor __inTake;

    DcMotor viperR;
    DcMotor viperL;


    int MAX_VIPER = 3000;
    int MIN_VIPER = 0;

    int time_sleep = 150;

    boolean status = true;

    boolean statusRotor = false;
    boolean statusPinca = false;
    boolean statusPulso = false;
    boolean statusBraco = false;

    @Override
    public void runOpMode() throws InterruptedException {
        RF  = hardwareMap.get(DcMotor.class, "M4");
        RB  = hardwareMap.get(DcMotor.class, "M3");
        LF  = hardwareMap.get(DcMotor.class, "M2");
        LB  = hardwareMap.get(DcMotor.class, "M1");
        rotor = hardwareMap.get(DcMotor.class, "M8");

        ServoOuttake  = hardwareMap.get(Servo.class ,"S1");
        bracoOUTake  = hardwareMap.get(Servo.class, "S2");
        ServoPulsoIntake  = hardwareMap.get(Servo.class ,"S3");
        pincaOUTake  = hardwareMap.get(Servo.class ,"S4");

        viperR = hardwareMap.get(DcMotor.class,"M6");
        viperL = hardwareMap.get(DcMotor.class,"M5");

        __inTake = hardwareMap.get(DcMotor.class,"M7");

        ServoOuttake.setPosition(-1);

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);




        waitForStart();


        while(opModeIsActive()){
            controle1();
            controle2();

        }

    }

    public void controle1(){
        movimentacao();
        extensor();
    }

    public void controle2(){
        PairMotor viper = new PairMotor();
        viper.setMotors(viperR,viperL);
        intake();
        outtake();
        viperComandos(viper);


    }

    public void viperComandos(PairMotor v){
            if(gamepad2.right_trigger > 0){
                v.sePMtPower(-gamepad2.right_trigger);

            } else if (gamepad2.left_trigger > 0){
                v.sePMtPower(gamepad2.left_trigger);

            } else {
                v.sePMtPower(0);
            }

    }


    public void movimentacao(){

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

        public void setMotors(DcMotor motor1, DcMotor motor2){
            m1 = motor1;
            m2 = motor2;
        }

        public void sePMtPower(double power){
            m1.setPower(power);
            m2.setPower(power);
        }

        public void sePMtPower(double power1, double power2){
            m1.setPower(power1);
            m2.setPower(power2);
        }


    }

    public void outtake() {



        if(gamepad2.y){
            statusBraco = !statusBraco;
        }

        if(statusBraco){
            bracoOUTake.setPosition(1);
        }else {
            bracoOUTake.setPosition(0);
        }

        if(gamepad2.a){
            statusPinca = !statusPinca;
        }

        if(statusPinca){
            pincaOUTake.setPosition(1);
        }else {
            pincaOUTake.setPosition(0);
        }


    }

    public void intake(){
        // Braço
        if (gamepad2.x){
            statusRotor = !statusRotor;
            sleep(time_sleep);
        }
        if (statusRotor){
            rotor.setPower(0.6);
        } else {
            rotor.setPower(-0.6);

        }



    }

    public void extensor(){
        if(gamepad1.right_stick_button){
            status = !status;
            sleep(200);
        }

        if(status){
            __inTake.setPower(-0.2);

        }else if(gamepad1.right_bumper){
            __inTake.setPower(0.5);
        }
        else if(gamepad1.left_bumper){
            __inTake.setPower(-0.5);
        }
        else {
            __inTake.setPower(0);
        }
    }







}