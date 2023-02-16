package org.firstinspires.ftc.teamcode.Autonomos;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Autonomos.Apriltag.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous(name = "AutonomoEncoder_DERECHA")
public class AutonomoEncoder_DERECHA extends LinearOpMode {

    DcMotor front_right;
    DcMotor front_left;
    DcMotor back_right;
    DcMotor back_left;
    DcMotor brazo;
    DcMotor elevador1;
    DcMotor elevador2;
    Servo garra;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    BNO055IMU imu;
    ElapsedTime timer2 = new ElapsedTime();
    double integralSum = 0;

    int position = 0;

    final double TICkS_PER_REV = 537.7;

    final double DRIVE_GEAR_REDUCTION = 1;
    final double WHEEL_DIAMETER_INCHES = 3.77953;
    double COUNTS_PER_INCH = (TICkS_PER_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    double ACCELERATION = 0.5;
    double MIN_POWER = 0.1;
    double DEACCELERATION_PERCENTAGE = 0.5;


    static final double FEET_PER_METER = 3.28084;


    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;
    int PARK1 = 1;
    int PARK2 = 2;
    int PARK3 = 3;

    public AprilTagDetection tagOfInterest;

    @Override
    public void runOpMode() throws InterruptedException {


        front_left = hardwareMap.get(DcMotor.class, "FL");
        front_right = hardwareMap.get(DcMotor.class, "FR");
        back_left = hardwareMap.get(DcMotor.class, "BL");
        back_right = hardwareMap.get(DcMotor.class, "BR");

        brazo = hardwareMap.dcMotor.get("brazo");//motor expansion 3
        elevador1 = hardwareMap.dcMotor.get("elev1");//motor expansion 0
        elevador2 = hardwareMap.dcMotor.get("elev2");//motor expansion 1
        garra = hardwareMap.servo.get("garra");//servo control 0

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters IMUparameters = new BNO055IMU.Parameters();
        IMUparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMUparameters);

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brazo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevador1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevador2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brazo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        garra.setPosition(0);

        elevador1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevador2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == PARK1 || tag.id == PARK2 || tag.id == PARK3) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                }

            }

            telemetry.update();
            sleep(20);
            cerrargarra();
        }
        /**---------------------------------------------------------
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         **/

        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        brazito(0.3, -45);
        detenerllantas(100);
        enfrente(0.5, 50.7);
        atras(0.3, 2);
        detenerllantas(300);
        brazito(0.3, -887);
        giroangulo(180 + 45);
        usar_elevador(0.4, 880);
        detenerllantas(800);
        enfrente(0.2, 10.8);
        abrirgarra();
        //Primer cono

        detenerllantas(900);
        atras(0.2, 11.8);
        usar_elevador(0.3, 380);
        detenerllantas(200);
        brazito(0.3, -2);
        giroangulo(180 - 91.5);
        enfrente(0.4, 26.5);
        cerrargarra();
        detenerelevadoor(450);
        usar_elevador(0.4, 900);
        detenerllantas(750);

        atras(0.4, 26.5);
        giroangulo(180 - 40);
        usar_elevador(0.4, 340);
        brazito(0.4, -1400);
        detenerllantas(400);
        atras(0.2, 3);
        detenerllantas(400);
        abrirgarra();
        //2do CONO

        detenerllantas(500);
        detenerelevadoor(300);
        enfrente(0.2, 4);
        usar_elevador(0.3, 30);
        giroangulo(180 + 5);
        brazito(0.3, -1);
        Derecha(0.2, 3);
        atras(0.4, 21.5);

        if(tagOfInterest.id == 1)//PRIMER PARK
        {
            Izquierda(0.4, 27);

            telemetry.addLine("1er park");
            telemetry.update();
        }
        else if(tagOfInterest.id == 2) //SEGUNDO PARK
        {
            telemetry.addLine("2do park");
            telemetry.update();
        }
        else if (tagOfInterest.id == 3)//3ER PARK
        {
            Derecha(0.4, 27);
            telemetry.addLine("3er park");
            telemetry.update();
        }
    }


    ElapsedTime timer = new ElapsedTime();

    public void encoderDrive(double speed, double fl, double fr, double bl, double br) {
        timer2.reset();

        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;

        newLeftFrontTarget = front_left.getCurrentPosition() + (int) (fl * COUNTS_PER_INCH);
        newRightFrontTarget = front_right.getCurrentPosition() + (int) (fr * COUNTS_PER_INCH);
        newRightBackTarget = back_right.getCurrentPosition() + (int) (br * COUNTS_PER_INCH);
        newLeftBackTarget = back_left.getCurrentPosition() + (int) (bl * COUNTS_PER_INCH);

        front_left.setTargetPosition(newLeftFrontTarget);
        front_right.setTargetPosition(newRightFrontTarget);
        back_right.setTargetPosition(newRightBackTarget);
        back_left.setTargetPosition(newLeftBackTarget);


        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (front_right.isBusy() && front_left.isBusy() && back_left.isBusy() && back_right.isBusy() && opModeIsActive()) {

            front_right.setPower(speed);
            front_left.setPower(speed);
            back_left.setPower(speed);
            back_right.setPower(speed);

            telemetry.addData("elev", elevador1.getCurrentPosition());
            telemetry.addData("elev2", elevador2.getCurrentPosition());
            telemetry.addData("Posicion del BL", back_left.getCurrentPosition());
            telemetry.addData("Posicion del BR", back_right.getCurrentPosition());
            telemetry.addData("Posicion del FR", front_right.getCurrentPosition());
            telemetry.addData("Posicion deL FL", front_left.getCurrentPosition());
            telemetry.update();

        }

        front_right.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        back_left.setPower(0);

        sleep(200);

        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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


    void elevador(double power, int ticks) {
        elevador1.setTargetPosition(ticks);
        elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevador1.setPower(power);

        elevador2.setTargetPosition(ticks);
        elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevador2.setPower(power);
    }

    void enfrente(double velocidad, double CantidadEnPulgadas) {
        encoderDrive(velocidad, CantidadEnPulgadas, CantidadEnPulgadas, CantidadEnPulgadas, CantidadEnPulgadas); //fl, fr, bl, br
    }

    void atras(double velocidad, double CantidadEnPulgadas) {
        encoderDrive(velocidad, -CantidadEnPulgadas, -CantidadEnPulgadas, -CantidadEnPulgadas, -CantidadEnPulgadas);
    }

    void Derecha(double velocidad, double CantidadEnPulgadas) {
        encoderDrive(velocidad, CantidadEnPulgadas, -CantidadEnPulgadas, -CantidadEnPulgadas, CantidadEnPulgadas);
    }

    void Izquierda(double velocidad, double CantidadEnPulgadas) {
        encoderDrive(velocidad, -CantidadEnPulgadas, CantidadEnPulgadas, CantidadEnPulgadas, -CantidadEnPulgadas);
    }

    void rotarIzq(double velocidad, double CantidadEnPulgadas) {
        encoderDrive(velocidad, -CantidadEnPulgadas, CantidadEnPulgadas, CantidadEnPulgadas, CantidadEnPulgadas);
    }

    void rotarDer(double velocidad, double CantidadEnPulgadas) {
        encoderDrive(velocidad, CantidadEnPulgadas, -CantidadEnPulgadas, -CantidadEnPulgadas, -CantidadEnPulgadas);
    }

    void diagIzqAr(double velocidad, double CantidadEnPulgadas) {
        encoderDrive(velocidad, 0, CantidadEnPulgadas, -CantidadEnPulgadas, 0);
    }

    void diagDerAr(double velocidad, double CantidadEnPulgadas) {
        encoderDrive(velocidad, CantidadEnPulgadas, 0, 0, CantidadEnPulgadas);
    }

    void diagIzqAt(double velocidad, double CantidadEnPulgadas) {
        encoderDrive(velocidad, -CantidadEnPulgadas, 0, 0, -CantidadEnPulgadas);
    }

    void diagDerAt(double velocidad, double CantidadEnPulgadas) {
        encoderDrive(velocidad, 0, -CantidadEnPulgadas, CantidadEnPulgadas, 0);
    }

    void usar_elevador(double velocidad, int posicion) {

        elevador1.setTargetPosition(posicion);
        elevador2.setTargetPosition(posicion);

        elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        elevador1.setPower(velocidad);
        elevador2.setPower(velocidad);

    }

    void brazito(double velocidad, int posicion) { //-1385 para subir atras
        brazo.setTargetPosition(posicion);
        brazo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brazo.setPower(velocidad);

    }

    void cerrargarra() {

        garra.setPosition(0);

    }

    void abrirgarra() {

        garra.setPosition(0.38);

    }

    public void detenerllantas(int tiempoMILISEGUNDOS) {

        front_left.setPower(0);
        back_left.setPower(0);
        front_right.setPower(0);
        back_right.setPower(0);
        sleep(tiempoMILISEGUNDOS);

    }

    public void detenerelevadoor(int tiempoMILISEGUNDOS) {

        elevador1.setPower(0);
        elevador2.setPower(0);
        sleep(tiempoMILISEGUNDOS);

    }

}