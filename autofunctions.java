package org.firstinspires.ftc.teamcode.programs2021;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import android.graphics.Color;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.util.ElapsedTime;

public class autofunctions
{
    private DcMotor motorL_Up;
    private DcMotor motorL_Down;
    private DcMotor motorR_Up;
    private DcMotor motorR_Down;

    private DcMotor BMotor;


    private Servo RedServo;
    private Servo BlackServo;

    private Servo BServo;

    private DcMotor ArmMotor_Left;
    private DcMotor ArmMotor_Right;

    private DcMotor IntakeMotor;

    private Servo armservo;

    private Servo shake_shack_servo;

    private Servo side_servo;

    private Servo side_servo_claw;

    private CRServo intakeservo;

    private DcMotor Side_Arm_Gang;

    int CurrentPosition = 0;

    int ArmPosition = 0;


    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU imu;

    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .7, correction;



    Orientation angles;

    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;
    NormalizedColorSensor colorSensor3;
    NormalizedColorSensor colorSensor4;

    NormalizedColorSensor CS1;
    NormalizedColorSensor CS2;

    Telemetry telemetry;

    public void Initialize(DcMotor motorL_DownIn,
                           DcMotor motorR_DownIn,
                           DcMotor motorR_UpIn,
                           DcMotor motorL_UpIn,
                           DcMotor BMotorIn,
                           Servo BServoIn,
                           CRServo intakeservoIn,

                           DcMotor ArmMotor_LeftIn,
                           DcMotor ArmMotor_RightIn,
                           DcMotor IntakeMotorIn,

                           BNO055IMU imuIn,

                           NormalizedColorSensor CS1In,
                           NormalizedColorSensor CS2In,

                           Telemetry telemetryIn)
    {
        motorL_Down = motorL_DownIn;
        motorR_Down = motorR_DownIn;
        motorR_Up = motorR_UpIn;
        motorL_Up = motorL_UpIn;
        BMotor = BMotorIn;
        BServo = BServoIn;
        intakeservo = intakeservoIn;


        ArmMotor_Left = ArmMotor_LeftIn;
        ArmMotor_Right = ArmMotor_RightIn;
        IntakeMotor = IntakeMotorIn;

        imu = imuIn;
        CS1 = CS1In;
        CS2 = CS2In;

        telemetry = telemetryIn;
    }

    public void ResetGyro()
    {
        //potential gyro, we will just let it stay here
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu.initialize(parameters);

        while ( !imu.isGyroCalibrated())
        {
            sleep(50);
        }
    }

    public void DriveForward (double Power, int Distance)
    {
        motorR_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        AllBRAKE();

        motorR_Up.setTargetPosition(Distance);
        motorR_Down.setTargetPosition(Distance);
        motorL_Up.setTargetPosition(-Distance);
        motorL_Down.setTargetPosition(-Distance);

        motorR_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        motorR_Up.setPower(Power);
        motorR_Down.setPower(Power);
        motorL_Up.setPower(-Power);
        motorL_Down.setPower(-Power);

        while (motorR_Down.isBusy()&& motorL_Down.isBusy()&&motorR_Up.isBusy()&&motorL_Up.isBusy())
        {
            //Wait until the task is done
        }
        StopDriving();
    }


    public void DriveBackGyro(double Power, int Distance)
    {
        ResetGyro();
        // up motor no encoder
        motorR_Up.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR_Up.setDirection(DcMotor.Direction.FORWARD);
        motorL_Up.setDirection(DcMotor.Direction.REVERSE);

        motorL_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        AllBRAKE();

        motorR_Down.setTargetPosition(Distance);
        motorL_Down.setTargetPosition(-Distance);


        motorL_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorL_Up.setPower(Power);
        motorR_Up.setPower(Power);

        motorL_Down.setPower(Power);
        motorR_Down.setPower(Power);



        while (motorR_Down.isBusy()&& motorL_Down.isBusy())
        {
            correction = checkDirection();
            motorL_Up.setPower(Power-correction);
            motorR_Up.setPower(Power+correction);
            //Wait until the task is done
        }
        StopDriving();

        motorR_Down.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR_Up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorL_Up.setDirection(DcMotor.Direction.FORWARD);

    }

    public void DriveForwardGyro(double Power, int Distance)
    {
        ResetGyro();
        // Down motor no encoder
        motorR_Down.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR_Down.setDirection(DcMotor.Direction.FORWARD);
        motorL_Down.setDirection(DcMotor.Direction.REVERSE);

        motorL_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        AllBRAKE();

        motorR_Up.setTargetPosition(Distance);
        motorL_Up.setTargetPosition(-Distance);


        motorL_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorL_Down.setPower(Power);
        motorR_Down.setPower(Power);

        motorL_Up.setPower(Power);
        motorR_Up.setPower(Power);



        while (motorR_Up.isBusy()&& motorL_Up.isBusy())
        {
            correction = checkDirection();
            motorL_Down.setPower(Power-correction);
            motorR_Down.setPower(Power+correction);
            //Wait until the task is done
        }
        StopDriving();

        motorR_Up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorL_Down.setDirection(DcMotor.Direction.FORWARD);

    }



    public void TurnLeft(double Power, int Distance)
    {


        motorR_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        AllBRAKE();

        motorR_Up.setTargetPosition(Distance);
        motorR_Down.setTargetPosition(Distance);
        motorL_Up.setTargetPosition(Distance);
        motorL_Down.setTargetPosition(Distance);

        motorR_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        motorR_Up.setPower(Power);
        motorR_Down.setPower(Power);
        motorL_Up.setPower(Power);
        motorL_Down.setPower(Power);

        while (motorR_Up.isBusy() && motorR_Down.isBusy()&& motorL_Up.isBusy()&& motorL_Down.isBusy())
        {
            //Wait until the task is done
        }

        StopDriving();
    }



    public void TurnRight(double Power, int Distance)
    {

        motorR_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        AllBRAKE();

        motorR_Up.setTargetPosition(-Distance);
        motorR_Down.setTargetPosition(-Distance);
        motorL_Up.setTargetPosition(-Distance);
        motorL_Down.setTargetPosition(-Distance);

        motorR_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        motorR_Up.setPower(-Power);
        motorR_Down.setPower(-Power);
        motorL_Up.setPower(-Power);
        motorL_Down.setPower(-Power);

        while (motorR_Up.isBusy() && motorR_Down.isBusy()&& motorL_Up.isBusy()&& motorL_Down.isBusy())
        {
            //Wait until the task is done
        }

        StopDriving();
    }

    public void StrafeRight(double Power, int Distance)
    {


        motorR_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        AllBRAKE();

        motorR_Up.setTargetPosition(-Distance);
        motorR_Down.setTargetPosition(Distance);
        motorL_Up.setTargetPosition(-Distance);
        motorL_Down.setTargetPosition(Distance);

        motorR_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        motorR_Up.setPower(-Power);
        motorR_Down.setPower(Power);
        motorL_Up.setPower(-Power);
        motorL_Down.setPower(Power);

        while (motorR_Up.isBusy() && motorR_Down.isBusy()&& motorL_Up.isBusy()&& motorL_Down.isBusy())
        {
            //Wait until the task is done
        }

        StopDriving();
    }

    public void StrafeRightGyro(double Power, int Distance)
    {
        correction = checkDirection();

        motorR_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        AllBRAKE();

        motorR_Up.setTargetPosition(-Distance);
        motorR_Down.setTargetPosition(Distance);
        motorL_Up.setTargetPosition(-Distance);
        motorL_Down.setTargetPosition(Distance);

        motorR_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        motorR_Up.setPower(-Power);
        motorR_Down.setPower(Power);
        motorL_Up.setPower(-Power);
        motorL_Down.setPower(Power);

        while (motorR_Up.isBusy() && motorR_Down.isBusy()&& motorL_Up.isBusy()&& motorL_Down.isBusy())
        {
            //Wait until the task is done
        }

        StopDriving();
    }

    public void StrafeLeft(double Power, int Distance)
    {


        motorR_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        AllBRAKE();

        motorR_Up.setTargetPosition(Distance);
        motorR_Down.setTargetPosition(-Distance);
        motorL_Up.setTargetPosition(Distance);
        motorL_Down.setTargetPosition(-Distance);


        motorR_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        motorR_Up.setPower(Power);
        motorR_Down.setPower(-Power);
        motorL_Up.setPower(Power);
        motorL_Down.setPower(-Power);

        while (motorR_Up.isBusy() && motorR_Down.isBusy()&& motorL_Up.isBusy()&& motorL_Down.isBusy())
        {
            //Wait until the task is done
        }

        StopDriving();
    }



    public void ArmUpDown    (double Power, int Distance)
    {


        ArmMotor_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ArmMotor_Left.setTargetPosition(-Distance);
        ArmMotor_Right.setTargetPosition(Distance);

        ArmMotor_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ArmMotor_Left.setPower(-Power);
        ArmMotor_Right.setPower(Power);

        while (ArmMotor_Left.isBusy() && ArmMotor_Right.isBusy())
        {
            //Wait until the task is done
        }

        StopDriving();
    }

    public void ArmUpDownTime    (double Power, int Distance, int Time)
    {
        ArmMotor_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ArmMotor_Left.setTargetPosition(-Distance);
        ArmMotor_Right.setTargetPosition(Distance);

        ArmMotor_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ArmMotor_Left.setPower(-Power);
        ArmMotor_Right.setPower(Power);

        sleep(Time);

        StopDriving();
    }

    public void BobberMotor (double Power, int Distance)
    {

        BMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        BMotor.setTargetPosition(Distance);

        BMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        BMotor.setPower(Power);

        while (BMotor.isBusy())
        {
            //wait until its done
        }

        BMotor.setPower(0);



    }


    public void IntakeTime (double Power, long Time)
    {
        ArmMotor_Left.setPower(-Power);
        ArmMotor_Right.setPower(Power);
        sleep(1069);
        IntakeMotor.setPower(-0.69);
        sleep(Time);
        ArmMotor_Left.setPower(0);
        ArmMotor_Right.setPower(0);
        IntakeMotor.setPower(0);
    }

    public void CloseServo ()
    {
        armservo.setPosition(1);
    }

    public void OpenServo ()
    {
        armservo.setPosition(0);
    }

    public void ServoDown ()
    {
        BlackServo.setPosition(-0.8);
        RedServo.setPosition(-0.8);
    }

    public void ServoUp ()
    {
        BlackServo.setPosition(0.8);
        RedServo.setPosition(0.8);
    }

    public void StopDriving ()
    {
        motorL_Up.setPower(0);
        motorL_Down.setPower(0);
        motorR_Up.setPower(0);
        motorR_Down.setPower(0);
    }

    public void AllFLOAT ()
    {
        motorR_Down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorL_Down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorR_Up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorL_Up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void AllBRAKE ()
    {
        motorR_Down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorL_Down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorR_Up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorL_Up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void getcube()
    {

        int Distance = 200;
        //Drive Backward To move arm up
        DriveForward(0.469, 650);
        sleep(250);

        //Bring Arm Up
        ArmUpDown(0.7, 2200);
        sleep(250);

        //Drive Forward
        DriveForward(0.469,-600);
        sleep(569);

        //Close the Servo
        CloseServo();
        sleep(569);

        ArmUpDown(1,-669);

        int got_turn = getTurn();
        if (got_turn == 1)
        {
            Distance += 0;
        }
        if (got_turn == 2)
        {
            Distance -= 50;
        }
        if (got_turn == 3)
        {
            Distance -= 150;
        }

        DriveForward(0.469, Distance);

    }

    public int getringcolor() {
        NormalizedRGBA colors = CS1.getNormalizedColors();
        NormalizedRGBA colors2 = CS2.getNormalizedColors();

        int color = colors.toColor();
        int color2 = colors2.toColor();

        float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
        colors.red /= max;
        colors.green /= max;
        colors.blue /= max;
        color = colors.toColor();

        float max2 = Math.max(Math.max(Math.max(colors2.red, colors2.green), colors2.blue), colors2.alpha);
        colors2.red /= max2;
        colors2.green /= max2;
        colors2.blue /= max2;
        color2 = colors2.toColor();


        telemetry.addLine("normalized color 18: ")
                .addData("a", "%d", Color.alpha(color))
                .addData("r", "%d", Color.red(color))
                .addData("g", "%d", Color.green(color))
                .addData("b", "%d", Color.blue(color));

        if (Color.red(color) < 100 && Color.red(color2) < 100) {
            telemetry.addLine("no rings");
            telemetry.addData("r", "%d", Color.red(color));
            telemetry.addData("r", "%d", Color.red(color2));

            telemetry.update();
            return 1; // 1 = true
        }
        if (Color.red(color) > 100 && Color.red(color2) < 100) {
            telemetry.addLine("1 ring");
            telemetry.addData("r", "%d", Color.red(color));
            telemetry.addData("r", "%d", Color.red(color2));

            telemetry.update();
            return 2; // 1 = true
        }
        if (Color.red(color) > 100 && Color.red(color2) > 100) {
            telemetry.addLine("3 rings");
            telemetry.addData("r", "%d", Color.red(color));
            telemetry.addData("r", "%d", Color.red(color2));

            telemetry.update();
            return 3; // 1 = true
        }

        telemetry.addLine("nope");
        telemetry.addData("r", "%d", Color.red(color));
        telemetry.addData("r", "%d", Color.red(color2));
        telemetry.update();
        return 0;
    }

    public void program() {
        int get_color = getringcolor();

        if (getringcolor()==1)
        {
            //DriveForward(0.3,-769);


            DriveForward(0.3, (-2669 + -769) );
            sleep(100);
            TurnLeft(0.3,1269);
            sleep(100);
            DriveForward(0.3,-500);
            sleep(100);
            BobberMotor(0.2,469);
            sleep(1000);
            BServo.setPosition(1);
            sleep(300);
           // BServo.setPosition(0);
            BobberMotor(0.5,-669);
            sleep(250);
            DriveForward(0.6,2769);
            sleep(100);
            TurnLeft(0.3,1269);
            sleep(100);
            DriveForward(0.6,-4038);
            sleep(100);
            TurnRight(0.3,1269);
            sleep(100);
            DriveForward(0.3,-800);
            sleep(100);
            TurnRight(0.3,769);
            sleep(100);
            DriveForward(0.1,-800);
            DriveForward(0.3,-3038);
            sleep(100);
            DriveForward(0.69,969);
            sleep(100);
            TurnRight(0.69,369);
            sleep(100);
            DriveForward(0.69,-1269);
        }
        if (getringcolor()==2)
        {
            DriveForward(0.3,-3069);
            sleep(100);
            BobberMotor(0.5,469);
            sleep(100);
            BServo.setPosition(1);
            sleep(1000);
            BServo.setPosition(0);
            BobberMotor(0.5,-469);
            sleep(100);
            DriveForward(0.5,4069);
            sleep(100);
            TurnRight(0.3,1269);
            sleep(100);
            DriveForward(0.3,-1369);
            sleep(100);
            TurnLeft(0.3,1669);
            sleep(100);
            DriveForward(0.3,-4169);

        }

    }


    public int getTurn()
    {
        if (CurrentPosition < 250 )
        {
            return 1;
        }
        if (CurrentPosition > 650)
        {
            return 3;
        }

        return 2;
    }

    public void TurnRightCompensation (double Power, int Distance)
    {
        motorR_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        AllBRAKE();

        int got_turn = getTurn();
        if (got_turn == 1)
        {
            Distance  += 25;
        }
        if (got_turn == 2)
        {
            Distance += 32  ;
        }
        if (got_turn == 3)
        {
            Distance += 69;
        }

        motorR_Up.setTargetPosition(-Distance);
        motorR_Down.setTargetPosition(-Distance);
        motorL_Up.setTargetPosition(-Distance);
        motorL_Down.setTargetPosition(-Distance);

        motorR_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        motorR_Up.setPower(-Power);
        motorR_Down.setPower(-Power);
        motorL_Up.setPower(-Power);
        motorL_Down.setPower(-Power);

        while (motorR_Up.isBusy() && motorR_Down.isBusy()&& motorL_Up.isBusy()&& motorL_Down.isBusy())
        {
            //Wait until the task is done
        }

        StopDriving();
    }

    public void TurnLeftCompensation (double Power, int Distance)
    {
        motorR_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        AllBRAKE();

        int got_turn = getTurn();
        if (got_turn == 1)
        {
            Distance  += 25;
        }
        if (got_turn == 2)
        {
            Distance += 32  ;
        }
        if (got_turn == 3)
        {
            Distance += 69;
        }

        telemetry.addData("turndistance %d", Distance);
        telemetry.update();

        motorR_Up.setTargetPosition(Distance);
        motorR_Down.setTargetPosition(Distance);
        motorL_Up.setTargetPosition(Distance);
        motorL_Down.setTargetPosition(Distance);

        motorR_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        motorR_Up.setPower(Power);
        motorR_Down.setPower(Power);
        motorL_Up.setPower(Power);
        motorL_Down.setPower(Power);

        while (motorR_Up.isBusy() && motorR_Down.isBusy()&& motorL_Up.isBusy()&& motorL_Down.isBusy())
        {

        }

        StopDriving();
    }

    //GYRO STUFF DONT TOUCH/REMOVE
    //GYRO STUFF DONT TOUCH/REMOVE
    //GYRO STUFF DONT TOUCH/REMOVE
    //GYRO STUFF DONT TOUCH/REMOVE
    //GYRO STUFF DONT TOUCH/REMOVE
    //GYRO STUFF DONT TOUCH/REMOVE
    //GYRO STUFF DONT TOUCH/REMOVE
    //GYRO STUFF DONT TOUCH/REMOVE
    //GYRO STUFF DONT TOUCH/REMOVE
    //GYRO STUFF DONT TOUCH/REMOVE
    //GYRO STUFF DONT TOUCH/REMOVE
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        globalAngle = 0;
    }

    private double getAngle()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private double checkDirection()
    {
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;
        else
            correction = -angle;

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        motorR_Up.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorL_Up.setDirection(DcMotor.Direction.REVERSE);


        // set power to rotate.
        motorL_Up.setPower(leftPower);
        motorR_Up.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (getAngle() == 0) {}

            while (getAngle() > degrees) {}
        }
        else    // left turn.
            while (getAngle() < degrees) {}

        // turn the motors off.
        motorR_Up.setPower(0);
        motorL_Up.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
}












