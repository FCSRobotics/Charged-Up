package frc.robot.utils;

import java.util.List;
import java.util.Queue;
import java.lang.Math;

public class LocationSensing {


    private double distance;

    private double angle;

    private double xpos;
    private double ypos;
    private double corrx;
    private double corry;

   

    public double[] getPosition(double area,double angle) {
        distance = Math.sqrt(area);
        xpos = distance*(Math.sin(-angle));
        ypos = distance*(Math.cos(-angle));
        corrx= 0.85*(-Math.cos(-angle)) + xpos;
        corry= 0.85*(-Math.sin(-angle)) + ypos;
        return new double[] {0.0,0.0};
      }

}

