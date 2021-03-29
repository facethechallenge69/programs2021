package org.firstinspires.ftc.teamcode.programs2021;
//needs to be fixed
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "69tele", group = "Tutorials")

//test zayn change

//test change amutha

public class
scrimtele extends LinearOpMode {
    private DcMotor motorL_Down;
    private DcMotor motorR_Down;
    private DcMotor motorR_Up;
    private DcMotor motorL_Up;

    private DcMotor ArmMotor1;
    private DcMotor ArmMotor2;

    private DcMotor BMotor;
    private Servo BServo;
    private DcMotor IntakeMotor;





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

        BMotor = hardwareMap.dcMotor.get("am");
        BServo = hardwareMap.servo.get("sg");
        IntakeMotor = hardwareMap.dcMotor.get("im");

        ArmMotor1 = hardwareMap.dcMotor.get("am1");
        ArmMotor2 = hardwareMap.dcMotor.get("am2");

        double motorSpeed = 1;

        double ThrowSpeed = 1;


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

        ArmMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            //moves wheels forward and backward

            if (gamepad1.a) {
                motorSpeed = 0.769;
            }

            if (gamepad1.b) {
                motorSpeed = 0.4;
            }

            if (gamepad1.x) {
                motorSpeed = 0.269;
            }

            if(gamepad2.a)
            {
               ThrowSpeed = 1;
            }
            if(gamepad2.b)
            {
                ThrowSpeed = 0.6;
            }
            if(gamepad2.x)
            {
                ThrowSpeed = 0.3;
            }
            if(gamepad2.y)
            {
                ThrowSpeed = 0.15;
            }


            //incremental movement for RedServo


//

            //moves mecanum wheels forward and backward

            if(moving == 0 && gamepad1.left_stick_y > 0 ) {
// should be forward
                moving = MOV_FRONT_BACK;
                motorL_Down.setPower(motorSpeed * -1);
                motorR_Down.setPower(motorSpeed * 1);
                motorL_Up.setPower(motorSpeed * -1);
                motorR_Up.setPower(motorSpeed * 1);
            }
            if(moving == 0 && gamepad1.left_stick_y < 0 ) {
// should be backward
                moving = MOV_FRONT_BACK;
                motorL_Down.setPower(motorSpeed * 1);
                motorR_Down.setPower(motorSpeed * -1);
                motorL_Up.setPower(motorSpeed * 1);
                motorR_Up.setPower(motorSpeed * -1);
            }
            else if(gamepad1.left_stick_y ==0 && moving == MOV_FRONT_BACK)
            {
                moving = 0;
            }


            //turns mecanum wheels left and right

            if(moving == 0 && gamepad1.left_stick_x > 0 ) {
// should be right
                moving = MOV_LEFT_RIGHT;
                motorL_Down.setPower(motorSpeed * -1);
                motorR_Down.setPower(motorSpeed * -1);
                motorL_Up.setPower(motorSpeed * -1);
                motorR_Up.setPower(motorSpeed * -1);
            }

            if(moving == 0 && gamepad1.left_stick_x < 0 ) {
// should be left
                moving = MOV_LEFT_RIGHT;
                motorL_Down.setPower(motorSpeed * 1);
                motorR_Down.setPower(motorSpeed * 1);
                motorL_Up.setPower(motorSpeed * 1);
                motorR_Up.setPower(motorSpeed * 1);
            }


            else if(gamepad1.left_stick_x ==0 && moving == MOV_LEFT_RIGHT)
            {
                moving = 0;
            }





            if (moving == 0 && gamepad1.dpad_right) {
                moving = STAF_RIGHT;
                motorL_Down.setPower(0.4 * -1);
                motorR_Down.setPower(0.4 * -1);
                motorL_Up.setPower(0.4 * 1);
                motorR_Up.setPower(0.4 * 1);
            }
            else if(!gamepad1.dpad_right && moving == STAF_RIGHT)
            {
                moving = 0;
            }



            if (moving == 0 && gamepad1.dpad_left) {
                moving = STRAF_LEFT;
                motorL_Down.setPower(0.4 * 1);
                motorR_Down.setPower(0.4 * 1);
                motorL_Up.setPower(0.4 * -1);
                motorR_Up.setPower(0.4 * -1);
            }

            else if(!gamepad1.dpad_left && moving == STRAF_LEFT)
            {
                moving = 0;
            }

            if(moving == 0)
            {
                motorL_Down.setPower(0);
                motorR_Down.setPower(0);
                motorL_Up.setPower(0);
                motorR_Up.setPower(0);
            }

            BMotor.setPower(0.2*gamepad1.right_stick_y);

            IntakeMotor.setPower(gamepad2.left_stick_y);

            ArmMotor1.setPower(-ThrowSpeed*gamepad2.right_stick_y);

            ArmMotor2.setPower(ThrowSpeed*gamepad2.right_stick_y);

            if(gamepad1.right_bumper)
            {
                BServo.setPosition(0);
            }

            if(gamepad1.left_bumper)
            {
                BServo.setPosition(1);
            }

        }


    }
}
/*

GamePad1:
Left Stick moves wheel
Right Stick moves the motor for the bobber
The left bumper opens the servo for the bobber, right bumper closes
a,b,x changes motor speed

GamePad2:
Left Stick moves the intake up and down
Right Stick controls the motors that throw the rings
a,b,x,y changes speed of the throw motors

*/