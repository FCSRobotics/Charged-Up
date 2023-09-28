package frc.robot.utils

class MovingAverage {
    private var numValues: Int
    private var values: DoubleArray
    private var total = 0.0
    private var index = 0

    constructor(numValues: Int) {
        this.numValues = numValues
        values = DoubleArray(numValues)
    }

    constructor(values: DoubleArray) {
        this.values = values
        numValues = values.size
    }

    fun setAll(value: Double) {
        total = value * numValues
        for (i in 0 until numValues) values[i] = value
    }

    fun addValue(value: Double): Double {
        index = index % numValues
        total -= values[index]
        total += value
        values[index] = value
        index++
        return total / numValues
    }

    val value: Double
        get() = total / numValues
}
