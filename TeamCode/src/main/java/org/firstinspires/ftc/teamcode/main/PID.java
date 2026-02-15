package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;

public class PID {
    public DcMotor motor;
    public PID(DcMotor pid_motor) {
        motor = pid_motor;
        DcMotorControllerEx motorControllerEx = (DcMotorControllerEx)motor.getController();
        
    }
    public void setRPM(int rpm) {
        float power = 0;
        
    };
}
