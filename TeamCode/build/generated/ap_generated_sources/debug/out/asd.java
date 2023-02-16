import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class asd extends OpMode {

    DcMotor front_left;
    DcMotor back_left;
    DcMotor front_right;
    DcMotor back_right;
    DcMotor brazo;
    DcMotor elevador1;
    DcMotor elevado2;
    Servo garra;


    @Override
    public void init() {

        front_left = hardwareMap.dcMotor.get("FL");
        back_left = hardwareMap.dcMotor.get("BL");
        front_right = hardwareMap.dcMotor.get("FR");
        back_right = hardwareMap.dcMotor.get("BR");
        garra = hardwareMap.servo.get("garra");
        brazo = hardwareMap.dcMotor.get("brazo");
        elevador1 = hardwareMap.dcMotor.get("elev1");
        elevado2 = hardwareMap.dcMotor.get("elev2");
        
        

        brazo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brazo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    @Override
    public void loop(){

        double posicion = brazo.getCurrentPosition();
        telemetry.addData("Posicion de esta vaina", posicion);

        elevador1.setPower(gamepad2.left_stick_y);
        elevado2.setPower(elevador1.getPower());

        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        double drive  = gamepad1.left_stick_y;
        double strafe = gamepad1.right_stick_x;
        double twist  = gamepad1.right_stick_y;

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

        // apply the calculated values to the motors.
        front_left.setPower(speeds[0]);
        front_right.setPower(speeds[1]);
        back_left.setPower(speeds[2]);
        back_right.setPower(speeds[3]);



            if(gamepad2.a){
                garra.setPosition(1);
            }else if (gamepad1.b) {
                garra.setPosition(0.5);
            }
            if (gamepad2.dpad_up){
                brazo.setTargetPosition(-278);
                brazo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                brazo.setPower(0.5);
            }else if (gamepad2.dpad_down){
                brazo.setTargetPosition(-1175);
                brazo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                brazo.setPower(0.5);
            }
    }
}