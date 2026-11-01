package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;

public final class DriveCommandMessage {
    public long timestamp;
    public double voltage;
    public double forwardVelocity;
    public double lateralVelocity;
    public double angularVelocity;

    public DriveCommandMessage(PoseVelocity2dDual<Time> vel) {
        this.timestamp = System.nanoTime();
        this.voltage = 0.0;
        this.forwardVelocity = vel.linearVel.x.value();
        this.lateralVelocity = vel.linearVel.y.value();
        this.angularVelocity = vel.angVel.value();
    }
}
