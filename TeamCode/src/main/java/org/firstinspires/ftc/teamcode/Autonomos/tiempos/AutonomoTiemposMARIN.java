package org.firstinspires.ftc.teamcode.Autonomos.tiempos;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
@Disabled
public class AutonomoTiemposMARIN extends LinearOpMode {
    DcMotorEx front_left;
    DcMotorEx back_left;
    DcMotorEx front_right;
    DcMotorEx back_right;
    DcMotorEx brazo;
    DcMotor elevador1;
    DcMotor elevador2;
    Servo garra;
    BNO055IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {
        front_left = hardwareMap.get(DcMotorEx.class,"FL");
        back_left = hardwareMap.get(DcMotorEx.class, "BL");
        front_right = hardwareMap.get(DcMotorEx.class, "FR");
        back_right = hardwareMap.get(DcMotorEx.class,"BR");
        garra = hardwareMap.servo.get("garra");
        brazo = hardwareMap.get(DcMotorEx.class,"brazo");
        elevador1 = hardwareMap.dcMotor.get("elev1");
        elevador2 = hardwareMap.dcMotor.get("elev2");

        BNO055IMU.Parameters IMUparameters = new BNO055IMU.Parameters();
        IMUparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMUparameters);

        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brazo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevador1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevador2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        brazo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevador2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevador1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brazo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevador1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevador2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Range.clip(brazo.getCurrentPosition(), -1385, 0);

        cerrargarra();

        waitForStart();

        subirelev1();
        enfrente(0.5, 2200);
        brazo(0.5 , -622);
        subirelev();
        giroangulo(34);
        enfrente(0.2, 1350);
        abrirgarra();
        sleep(300);
        atras(0.2, 1350);
        giroangulo(0);

        //AQUI PONE EL PRIMER CONO

        /*sleep(300);
        atras(0.2,800);
        giroangulo(268);
        bajarelev();
        brazo(0.5, -220);
        enfrente(0.25, 2900);
        cerrargarra();
        subirelev();
        sleep(700);
        brazo(0.5,-1385);
        atras(0.5,1000);
        giroangulo(235);
        atras(0.4,300);
    */
    }


    void enfrente(double potencia, int tiempoMILISEGNDOS) {
        front_right.setPower(potencia * 0.92);
        front_left.setPower(potencia);
        back_left.setPower(potencia * 0.92);
        back_right.setPower(potencia * 0.92);
        sleep(tiempoMILISEGNDOS);
        front_right.setPower(0);
        front_left.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
        sleep(100);

    }

    void atras(double potencia, int tiempoMILISEGUNDOS) {
        front_right.setPower(-potencia);
        front_left.setPower(-potencia);
        back_left.setPower(-potencia);
        back_right.setPower(-potencia);
        sleep(tiempoMILISEGUNDOS);
        front_right.setPower(0);
        front_left.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
        sleep(100);
    }

    void derecha(double potencia, int tiempoMILISEGUNDOS) {
        front_right.setPower(-potencia * 0.95);
        front_left.setPower(potencia);
        back_left.setPower(-potencia * 0.95);
        back_right.setPower(potencia * 0.95);
        sleep(tiempoMILISEGUNDOS);
        front_right.setPower(0);
        front_left.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
        sleep(100);
    }

    void izquierda(double potencia, int tiempoMILISEGUNDOS) {
        front_right.setPower(potencia * 0.95);
        front_left.setPower(-potencia);
        back_left.setPower(potencia * 0.95);
        back_right.setPower(-potencia * 0.95);
        sleep(tiempoMILISEGUNDOS);
        front_right.setPower(0);
        front_left.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
        sleep(100);
    }

    void giroangulo(double Angulo) {

        double Error = ObtenerError(Angulo);

        if (Error > 0) {

            giroderecha(0.4);
            while (Error > 0) {

                Error = ObtenerError(Angulo);

            }
            podermotor(0);

            giroizquierda(0.2);
            while (Error < 0) {

                Error = ObtenerError(Angulo);

            }
            podermotor(0);


        } else if (Error < 0) {

            giroizquierda(0.4);

            while (Error < 0) {
                Error = ObtenerError(Angulo);

            }
            giroizquierda(0);

            giroderecha(0.2);
            while (Error > 0) {

                Error = ObtenerError(Angulo);

            }
            podermotor(0);

        }
    }

    public double ObtenerAngulo() {
        Orientation Angulos;
        Angulos = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double PrimerAngulo = Angulos.firstAngle;
        PrimerAngulo = Angulos.firstAngle;
        return PrimerAngulo;

    }

    public double ObtenerError(double Angulo) {

        double Error = Angulo - ObtenerAngulo();
        if (Error > 180) Error -= 360;
        if (Error < -180) Error += 360;
        return Error;

    }

    void diagonalderechaenfrente(double potencia, int tiempoMILISEGUNDOS) {
        front_left.setPower(potencia);
        back_right.setPower(potencia * 0.95);
        sleep(tiempoMILISEGUNDOS);
        front_right.setPower(0);
        front_left.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
        sleep(100);
    }

    void diagonalizquierdaenfrente(double potencia, int tiempoMILISEGUNDOS) {
        front_right.setPower(potencia);
        back_left.setPower(potencia);
        sleep(tiempoMILISEGUNDOS);
        front_right.setPower(0);
        front_left.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
        sleep(100);
    }

    void diagonalizquierdaatras(double potencia, int tiempoMILISEGUNDOS) {
        front_left.setPower(-potencia);
        back_right.setPower(-potencia * 0.95);
        sleep(tiempoMILISEGUNDOS);
        front_right.setPower(0);
        front_left.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
        sleep(100);
    }

    void diagonalderechaatras(double potencia, int tiempoMILISEGUNDOS) {
        front_right.setPower(-potencia);
        back_left.setPower(-potencia);
        sleep(tiempoMILISEGUNDOS);
        front_right.setPower(0);
        front_left.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
        sleep(100);
    }

    void giroizquierda(double potencia) {
        front_right.setPower(potencia * 0.88);
        front_left.setPower(-potencia);
        back_left.setPower(-potencia);
        back_right.setPower(potencia * 0.88);

    }

    void giroderecha(double potencia) {
        front_right.setPower(-potencia);
        front_left.setPower(potencia * 0.88);
        back_left.setPower(potencia * 0.88);
        back_right.setPower(-potencia);

    }

    void podermotor(double potencia) {

        front_right.setPower(potencia);
        front_left.setPower(potencia);
        back_left.setPower(potencia);
        back_right.setPower(potencia);

    }

    void elevador (int target) {
        elevador1.setTargetPosition(target);
        elevador2.setTargetPosition(-target);

        elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        elevador1.setPower(0.4);
        elevador2.setPower(0.4);

    }


    void subirelev() {
        elevador1.setTargetPosition(1420);
        elevador2.setTargetPosition(-1420);

        elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        elevador1.setPower(1);
        elevador2.setPower(1);


    }
    void subirelev1() {
        elevador1.setTargetPosition(200);
        elevador2.setTargetPosition(-200);

        elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        elevador1.setPower(1);
        elevador2.setPower(1);


    }

    void bajarelev() {

        elevador1.setTargetPosition(0);
        elevador2.setTargetPosition(0);

        elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        elevador1.setPower(1);
        elevador2.setPower(1);

    }
    void brazo(double poder, int posicion) { //-1385 subir para atras
        brazo.setTargetPosition(posicion); //Antes: 1175
        brazo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brazo.setPower(poder);

    }


    void cerrargarra() {

        garra.setPosition(0);

    }

    void abrirgarra() {

        garra.setPosition(0.72);

    }

    void detenertodo(int tiempoMILISEGUNDOS) {

        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
        elevador2.setPower(0);
        elevador1.setPower(0);
        brazo.setPower(0);
        sleep(tiempoMILISEGUNDOS);

    }

}