package org.firstinspires.ftc.teamcode.Autonomos.Apriltag;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous(name = "Apriltag")
@Disabled
public class Apriltag_prueba_autonomo extends LinearOpMode {

    DcMotor front_right;
    DcMotor front_left;
    DcMotor back_right;
    DcMotor back_left;
    DcMotor brazo;
    DcMotor elevador1;
    DcMotor elevador2;
    Servo garra;

    BNO055IMU imu;
    ElapsedTime timer2 = new ElapsedTime();
    double integralSum = 0;

    int position = 0;

    final double TICkS_PER_REV = 537.6;

    final double DRIVE_GEAR_REDUCTION = 1;
    final double WHEEL_DIAMETER_INCHES = 3.77953;
    double COUNTS_PER_INCH = (TICkS_PER_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    double ACCELERATION = 0.5;
    double MIN_POWER = 0.1;
    double DEACCELERATION_PERCENTAGE = 0.5;

    ElapsedTime runtime = new ElapsedTime();

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

    private AprilTagDetection tagOfInterest = null;

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

        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

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

        garra.setPosition(0.7);

        elevador1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevador2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int position = 0;

        telemetry.addData("elev", elevador1.getCurrentPosition());
        telemetry.addData("elev2", elevador2.getCurrentPosition());
        telemetry.update();

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

        waitForStart();
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
        }
        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
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

        /* Actually do something useful */
        if(tagOfInterest.id == 1)//PRIMER PARK
        {

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
            telemetry.addLine("3er park");
            telemetry.update();
        }
        else //Si no jala ni uno
        {

        }


        while(opModeIsActive()){sleep(20);}

        /**Aqui va el autonomo
         Ejemplo:
         **/



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


        double currentPower = 0;
        boolean accelerationHasFinished = false;

        while (front_right.isBusy() && front_left.isBusy() && back_left.isBusy() && back_right.isBusy() && opModeIsActive()) {
            currentPower = Math.abs(currentPower);
            if (currentPower < speed && !accelerationHasFinished) {
                currentPower = timer2.seconds() * ACCELERATION;
            } else {
                accelerationHasFinished = true;
            }

            front_right.setPower(currentPower);
            front_left.setPower(currentPower);
            back_left.setPower(currentPower);
            back_right.setPower(currentPower);

        }

        front_right.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        back_left.setPower(0);

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    }



