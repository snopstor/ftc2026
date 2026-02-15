package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.hardware.DcMotor;

public class PID {
    public DcMotor motor;
    public PID(DcMotor pid_motor) {
        motor = pid_motor;
    }
    public void setRPM(int rpm) {
        
    };
}
