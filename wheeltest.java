package org.firstinspires.ftc.teamcode.programs2021;
//needs to be fixed
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "v6.9", group = "Tutorials")

//test zayn change

//test change amutha

public class
wheeltest extends LinearOpMode {
    private DcMotor motorL_Down;
    private DcMotor motorR_Down;
    private DcMotor motorR_Up;
    private DcMotor motorL_Up;


    private Servo RedServo;
    private Servo BlackServo;

    private DcMotor ArmMotor_Left;
    private DcMotor ArmMotor_Right;

    private Servo armservo;
    private Servo shake_shack_servo;


    int MOV_LEFT_RIGHT = 1;
    int MOV_FRONT_BACK = 2;
    int STRAF_LEFT = 3;
    int STAF_RIGHT = 4;

    //private Telemetry telemetry;


    @Override
    public void runOpMode() throws InterruptedException {


        motorL_Down = hardwareMap.dcMotor.get("left_motor_d");
        motorR_Down = hardwareMap.dcMotor.get("right_motor_d");
        motorL_Up = hardwareMap.dcMotor.get("left_motor_up");
        motorR_Up = hardwareMap.dcMotor.get("right_motor_up");


      /*  RedServo = hardwareMap.servo.get("red_servo");
        BlackServo = hardwareMap.servo.get("black_servo");

        ArmMotor_Left = hardwareMap.dcMotor.get("armmotor_l");
        ArmMotor_Right = hardwareMap.dcMotor.get("armmotor_r");

        armservo = hardwareMap.servo.get("arm_servo");
        shake_shack_servo = hardwareMap.servo.get("servo_arm");

/*
       */

        double motorSpeed = 1;

        String telly =  "" + motorSpeed;


        int moving = 0;

        double red_value = 0;

        double black_value = 1;

        double arm_servo = 0.5;

        double armPower = 0.1769;

        int ArmPosition = 0;

        double ShakeServo = 0;






        motorR_Down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorL_Down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorR_Up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorL_Up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      /*  ArmMotor_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        BlackServo.setPosition(1);
        RedServo.setPosition(1);

        ArmMotor_Right.getCurrentPosition();

*/

        waitForStart();

        //  ArmMotor_Right.getCurrentPosition();



        while (opModeIsActive()) {
            //moves wheels forward and backward

            if (gamepad1.a) {
                motorSpeed = 1;
            }

            if (gamepad1.b) {
                motorSpeed = 0.8;
            }

            if (gamepad1.x) {
                motorSpeed = 0.6;
            }

            if (gamepad1.dpad_up) {
            motorSpeed = 0.4;
            }


            if (gamepad1.y) {
                motorSpeed -= 0.025;
                sleep(400);

            }

            motorL_Up.setPower(gamepad1.left_stick_y * -motorSpeed);
            motorR_Up.setPower(gamepad1.left_stick_y * motorSpeed);
            motorL_Down.setPower(gamepad1.left_stick_y * -motorSpeed);
            motorR_Down.setPower(gamepad1.left_stick_y * motorSpeed);


            telly = "" + motorSpeed;


            // System.out.println(motorSpeed);

        //    System.out.flush();
            // telemetry.addLine("Got black");
            telemetry.addData("motorSeed1.0: %f", motorSpeed);
            // telemetry.addData("r", "%d", Color.red(color2));
         //   telemetry.addData("motorSeed3.0: ","%d", motorSpeed);

            telemetry.addData("motorSpeed2.0: %d", motorSpeed);



            telemetry.addData("servoposition %d", motorSpeed);

            telemetry.addData(telly, motorSpeed);

            telemetry.update();














        }


    }

    //void for claw
  /*  public void ClawForward(double Power, int Distance) {


        //Reset Encoders
        motorArmClaw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Set to RUN_TO_POSITION Mode
        motorArmClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //Set Target Position
        motorArmClaw.setTargetPosition(Distance);


        //Setting Motor Power
        motorArmClaw.setPower(Power);


        //While Loop to Make Sure Encoders do Not Deactivate
        while (motorArmClaw.isBusy()) {
            //Wait until the task is done
        }


    }
   */
    //void for Circ


}