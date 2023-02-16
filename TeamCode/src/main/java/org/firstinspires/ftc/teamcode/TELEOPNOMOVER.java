package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;

@TeleOp
public class TELEOPNOMOVER extends OpMode {

    DcMotor front_left;
    DcMotor back_left;
    DcMotor front_right;
    DcMotor back_right;
    DcMotor brazo;
    DcMotor elevador1;
    DcMotor elevador2;
    Servo garra;
    boolean modomanual = false;
    BNO055IMU imu;



    @Override
    public void init() {

        front_left = hardwareMap.dcMotor.get("FL");
        back_left = hardwareMap.dcMotor.get("BL");
        front_right = hardwareMap.dcMotor.get("FR");
        back_right = hardwareMap.dcMotor.get("BR");
        garra = hardwareMap.servo.get("garra");
        brazo = hardwareMap.dcMotor.get("brazo");
        elevador1 = hardwareMap.dcMotor.get("elev1");
        elevador2 = hardwareMap.dcMotor.get("elev2");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        brazo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brazo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        brazo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevador1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevador2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevador2.setDirection(DcMotorSimple.Direction.REVERSE);

        elevador2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevador2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevador1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevador1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        garra.setPosition(0.7);
    }

    @Override
    public void loop(){

        double BL = back_left.getCurrentPosition();
        telemetry.addData("Posicion del BL", back_left.getCurrentPosition());

        double BR = back_right.getCurrentPosition();
        telemetry.addData("Posicion del BR", back_right.getCurrentPosition());

        double FR = front_right.getCurrentPosition();
        telemetry.addData("Posicion del FR", front_right.getCurrentPosition());

        double FL = front_left.getCurrentPosition();
        telemetry.addData("Posicion deL FL", front_left.getCurrentPosition());

       double elev1 = elevador1.getCurrentPosition();
        telemetry.addData("Posicion deL elev1", elev1);

        double elev2 = elevador2.getCurrentPosition();
        telemetry.addData("Posicion deL elev2", elev2);

        double brazoa = brazo.getCurrentPosition();
        telemetry.addData("Posicion deL brazo", brazo);

        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        double drive  = -gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;
        double twist  = -gamepad1.left_stick_y;

        /*
         * If we had a gyro and wanted to do field-oriented control, here
         * is where we would implement it.
         *
         * The idea is fairly simple; we have a robot-oriented Cartesian (x,y)2
         * coordinate (strafe, drive), and we just rotate it by the gyro
         * reading minus the offset that we read in the init() method.
         * Some rough pseudocode demonstrating:
         *
         * if Field Oriented Control:
         *     get gyro heading
         *     subtract initial offset from heading
         *     convert heading to radians (if necessary)
         *     new strafe = strafe * cos(heading) - drive * sin(heading)
         *     new drive  = strafe * sin(heading) + drive * cos(heading)
         *
         * If you want more understanding on where these rotation formulas come
         * from, refer to
         * https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
         */

        double[] speeds = {
                (drive + strafe + twist), //rx + lx + ly
                (drive - strafe - twist), //rx - lx + ly
                (drive - strafe + twist), //rx - lx - ly
                (drive + strafe - twist) // rx + lx - ly


                //  double drive  = gamepad1.right_stick_x;
                //        double strafe = gamepad1.left_stick_x;
                //        double twist  = gamepad1.left_stick_y;
        };

        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }
        if(modomanual) {
            front_left.setPower(speeds[0]*0.5);
            front_right.setPower(speeds[1]*0.5);
            back_left.setPower(speeds[2]*0.5);
            back_right.setPower(speeds[3]*0.5);
            telemetry.addLine("Modo manual activado");
        }else if(!modomanual) {
    front_left.setPower(speeds[0]);
    front_right.setPower(speeds[1]);
    back_left.setPower(speeds[2]);
    back_right.setPower(speeds[3]);
    telemetry.addLine("Modo manual desactivado");
}

            elevador1.setPower(gamepad2.left_stick_y);
            elevador2.setPower(elevador1.getPower());
        if(gamepad1.right_trigger>0.5){
            modomanual=true;
        }else {
            modomanual=false;
        }

        Range.clip(brazo.getCurrentPosition(), -1385, 0);

            if (gamepad2.a) {
                garra.setPosition(0);
                brazo.setTargetPosition(-380);
                brazo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                brazo.setPower(0.3);

            } else if (gamepad2.b) {
                garra.setPosition(0.72);
            }
                if (gamepad2.dpad_up) {
                    brazo.setTargetPosition(-380); //Antes: 278
                    brazo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    brazo.setPower(0.3);
                }
                if (gamepad2.dpad_down) {
                    brazo.setTargetPosition(-2);
                    brazo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    brazo.setPower(0.2);
                }
                if (gamepad2.dpad_right) {
                    brazo.setTargetPosition(-1385); //Antes: 1175
                    brazo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    brazo.setPower(0.2);
                }



        /* AUTODESTRUCCION */
        if (gamepad1.left_trigger > 0.9 && gamepad1.right_trigger > 0.9 && gamepad2.left_trigger > 0.9 && gamepad2.right_trigger > 0.9){
            terminateOpModeNow();}
                }

    }




