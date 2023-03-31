package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;


public class MotorSpeedsSupplier {

    private List<MotorSpeed> motorSpeedQueue;

    public MotorSpeedsSupplier() {
        motorSpeedQueue = new ArrayList<MotorSpeed>();
    }       

    public double getMotorSpeed() {
        if(motorSpeedQueue.size() < 1) return 0;
        int highestPriority = motorSpeedQueue.get(0).priority;
        int highestPriorityIndex = 0;
        List<MotorSpeed> oldValues  = new ArrayList<>(motorSpeedQueue);
        for (int i = 0; i < oldValues.size(); i++) {
            MotorSpeed motorSpeed = oldValues.get(i);
            if(motorSpeed.priority < highestPriority) {
                highestPriority = motorSpeed.priority;
                highestPriorityIndex = i;
            } else if (motorSpeed.priority == highestPriority) {
                motorSpeedQueue.remove(highestPriorityIndex);
                highestPriority = motorSpeed.priority;
                highestPriorityIndex = i;
            }
        }

        return motorSpeedQueue.get(highestPriorityIndex).speed;
    }
    

    public void setSpeedWithPriority(double speed, int priority) {
        motorSpeedQueue.add(new MotorSpeed(speed, priority));
    }
}
