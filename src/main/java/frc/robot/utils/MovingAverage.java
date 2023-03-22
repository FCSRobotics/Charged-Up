package frc.robot.utils;

import java.util.List;
import java.util.Queue;

public class MovingAverage {
    private int numValues;

    private double[] values;

    private double total;

    private int index;

    public MovingAverage(int numValues) {
        this.numValues = numValues;
        this.values = new double[numValues];
    }

    public MovingAverage(double[] values) {
        this.values = values;
        numValues = values.length;
    }


    public double addValue(double value) {
        this.index = this.index % this.numValues;
        

        this.total -= this.values[this.index];

        this.total += value;
        
        this.values[this.index] = value;
        this.index++;


        return this.total / this.numValues;


    }

    public double getValue() {
        return this.total / this.numValues;
    }

    
}
