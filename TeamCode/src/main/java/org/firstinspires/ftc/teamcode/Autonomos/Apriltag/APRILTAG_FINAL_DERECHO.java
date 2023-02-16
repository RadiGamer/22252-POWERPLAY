
package org.firstinspires.ftc.teamcode.Autonomos.Apriltag;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "APRILTAG_LADO_DERECHO")
@Disabled
public class APRILTAG_FINAL_DERECHO extends LinearOpMode
{
    DcMotorEx front_left;
    DcMotorEx back_left;
    DcMotorEx front_right;
    DcMotorEx back_right;
    DcMotorEx brazo;
    DcMotor elevador1;
    DcMotor elevador2;
    Servo garra;
    BNO055IMU imu;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
    int PARK1 = 1;
    int PARK2 = 2;
    int PARK3 = 3;

    public AprilTagDetection tagOfInterest;

    @Override
    public void runOpMode()
    {
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

        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Range.clip(brazo.getCurrentPosition(), -1385, 0);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == PARK1 || tag.id == PARK2 || tag.id == PARK3 )
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
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

        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        subirelev1();
        enfrente(0.5, 2200);
        brazo(0.5 , -622);
        subirelev();
        giroangulo(-39);
        enfrente(0.3, 550);
        enfrente(0.2,450);
        abrirgarra();
        sleep(500);
        atras(0.3, 850);
        giroangulo(0);
        bajarelev();
        brazo(0.5, 0);
        atras(0.4, 900);

        if(tagOfInterest.id == 1)//PRIMER PARK
        {
            izquierda(0.5, 1500);

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
            derecha(0.5, 1300);
            telemetry.addLine("3er park");
            telemetry.update();
        }
        else //Si no jala ni uno
        {
            telemetry.addLine("No se detecto ninguno, estacionando en 2");
            telemetry.update();
        }
    }
    void enfrente(double potencia, int tiempoMILISEGNDOS) {
        front_right.setPower(potencia);
        front_left.setPower(potencia);
        back_left.setPower(potencia);
        back_right.setPower(potencia);
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

        garra.setPosition(0.37);

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
    void derecha(double potencia, int tiempoMILISEGUNDOS) {
        front_right.setPower(-potencia);
        front_left.setPower(-potencia);
        back_left.setPower(potencia);
        back_right.setPower(potencia);
        sleep(tiempoMILISEGUNDOS);
        front_right.setPower(0);
        front_left.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
        sleep(100);
    }

    void izquierda(double potencia, int tiempoMILISEGUNDOS) {
        front_right.setPower(potencia);
        front_left.setPower(potencia);
        back_left.setPower(-potencia);
        back_right.setPower(-potencia);
        sleep(tiempoMILISEGUNDOS);
        front_right.setPower(0);
        front_left.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
        sleep(100);
    }
}