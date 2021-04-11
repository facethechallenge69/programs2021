package org.firstinspires.ftc.teamcode.programs2021;
//needs to be fixed

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Mr. Guzman es mi negro", group = "Tutorials")

public class
servotest extends LinearOpMode
{
    private DcMotor motorL_Down;
    private DcMotor motorR_Down;
    private DcMotor motorR_Up;
    private DcMotor motorL_Up;

    private DcMotor motor_bobber;
    private Servo bobberclaw;

    @Override
    public void runOpMode() throws InterruptedException
    {

        /*motorL_Down = hardwareMap.dcMotor.get("left_motor_d");
        motorR_Down = hardwareMap.dcMotor.get("right_motor_d");
        motorL_Up = hardwareMap.dcMotor.get("left_motor_up");
        motorR_Up = hardwareMap.dcMotor.get("right_motor_up");*/

        motor_bobber = hardwareMap.dcMotor.get("Mr.Guzman");
        bobberclaw = hardwareMap.servo.get("senorguzman");

        double motorSpeed = 1;

        double grabberservoPosition = 0.5;

        int bobbermotorTicks = 0;

        motor_bobber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive())
        {

            BobberMotor();

            if (gamepad2.y)
            {
                grabberservoPosition += 0.05;
                sleep(400);
            }

            if (gamepad2.a)
            {
                grabberservoPosition -= 0.05;
                sleep(400);
            }

            if (gamepad2.x)
            {
                bobberclaw.setPosition(grabberservoPosition);
            }
        }
    }

    public int BobberMotor ()
    {

        motor_bobber.setPower(-0.2);

        motor_bobber.getTargetPosition();

        motor_bobber.getCurrentPosition();

        motor_bobber.setTargetPosition(motor_bobber.getTargetPosition());


        while (motor_bobber.isBusy())
        {
            //Wait until the task is done
        }

        motor_bobber.setPower(0);

        return motor_bobber.getTargetPosition();
    }
}