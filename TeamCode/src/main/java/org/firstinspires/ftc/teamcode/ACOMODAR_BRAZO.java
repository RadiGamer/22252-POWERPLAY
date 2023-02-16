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
public class ACOMODAR_BRAZO extends OpMode {


    DcMotor brazo;

    @Override
    public void init() {

        brazo = hardwareMap.dcMotor.get("brazo");

        brazo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brazo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        brazo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
    }

    @Override
    public void loop(){


        telemetry.addData("Posicion deL brazo", brazo.getCurrentPosition());

            brazo.setPower(gamepad2.right_trigger * 0.5);

        /* AUTODESTRUCCION */
        if (gamepad1.left_trigger > 0.9 && gamepad1.right_trigger > 0.9 && gamepad2.left_trigger > 0.9 && gamepad2.right_trigger > 0.9){
            terminateOpModeNow();}
    }

}



