package frc.robot.utils

class LocationSensing {
    private var distance = 0.0
    private val angle = 0.0
    private var xpos = 0.0
    private var ypos = 0.0
    private var corrx = 0.0
    private var corry = 0.0
    fun getPosition(area: Double, angle: Double): DoubleArray {
        distance = Math.sqrt(area)
        xpos = distance * Math.sin(-angle)
        ypos = distance * Math.cos(-angle)
        corrx = 0.85 * -Math.cos(-angle) + xpos
        corry = 0.85 * -Math.sin(-angle) + ypos
        return doubleArrayOf(0.0, 0.0)
    }
}
