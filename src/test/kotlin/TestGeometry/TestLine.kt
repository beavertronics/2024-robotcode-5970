package TestGeometry

import frc.engine.utils.Sugar.eqEpsilon
import frc.engine.utils.Units.Angular.AngleUnit
import frc.engine.utils.Units.Linear.feet
import frc.engine.utils.geometry.Line
import frc.engine.utils.geometry.Raycast2D
import frc.engine.utils.geometry.Vector2
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test
import kotlin.math.PI
import kotlin.math.absoluteValue

class TestLine {
    fun assertEpsilon(expected: Double, actual: Double, epsilon: Double = 0.01) {
        if (!(actual eqEpsilon expected)) {
            Assertions.fail<Double>("Assertion failed! expected: $expected found: $actual")
        }
        else Assertions.assertTrue((expected - actual).absoluteValue < epsilon)
    }
    fun assertEpsilon(expected: Vector2, actual: Vector2?, epsilon: Double = 0.01) {
        if(actual == null) Assertions.fail<Double>("Assertion failed! expected: $expected found: $actual")
        else if (!(actual.x eqEpsilon expected.x && actual.y eqEpsilon expected.y)) {
            Assertions.fail<Double>("Assertion failed! expected: $expected found: $actual")
        }
        else Assertions.assertTrue(true)
    }
    @Test
    fun testPerpendicularHorizontalIntersection(){
        val newLine = Line(
            Vector2.Companion.new(0.feet, 0.feet),
            Vector2.Companion.new(2.feet, 0.feet)
        )
        val Raycast = Raycast2D(Vector2.Companion.new(1.feet, 1.feet), AngleUnit.down)
        val intersection = newLine.intersection(Raycast)
        //println(intersection)
        assertEpsilon(Vector2.Companion.new(1.feet, 0.feet), intersection)
    }

    @Test
    fun testPerpendicularVerticalIntersection(){
        val newLine2 = Line(
            Vector2(0.0, 0.0),
            Vector2(0.0, 2.0)
        )

        val Raycast2 = Raycast2D(Vector2.new(1, 1), AngleUnit.left)
        val intersection2 = newLine2.intersection(Raycast2)
        assertEpsilon(Vector2.new(0, 1), intersection2)
    }
    @Test
    fun testPerpendicularVerticalIntersectionRight(){
        val newLine2 = Line(
            Vector2(2.0, 0.0),
            Vector2(2.0, 2.0)
        )

        val Raycast2 = Raycast2D(Vector2.new(1, 1), AngleUnit.right)
        val intersection2 = newLine2.intersection(Raycast2)
        assertEpsilon(Vector2.new(2, 1), intersection2)
    }
    @Test
    fun testNoIntersection(){
        val newLine2 = Line(
            Vector2(0.0, 0.0),
            Vector2(2.0, 2.0)
        )
        val Raycast2 = Raycast2D(Vector2.new(1, 0), AngleUnit.down)
        val intersection2 = newLine2.intersection(Raycast2) ?: run { Assertions.assertTrue(true); return }
        Assertions.fail<Double>("Assertion failed! expected: null found: $intersection2")
    }
    @Test
    fun testPerpendicularDiagonalIntersection(){
        val actual = Vector2.new(1, 1)
        val newLine2 = Line(
            Vector2(0.0, 0.0),
            Vector2(2.0, 2.0)
        )

        val Raycast2 = Raycast2D(Vector2.new(2, 0), AngleUnit((3 * PI) / 4))
        val intersection2 = newLine2.intersection(Raycast2)
        assertEpsilon(actual, intersection2)
    }
    @Test
    fun testDiagonalIntersection(){
        val actual = Vector2.new(1, 1)
        val newLine2 = Line(
            Vector2(0.0, 0.0),
            Vector2(2.0, 2.0)
        )

        val Raycast2 = Raycast2D(Vector2.new(1, 0), AngleUnit.up)
        val intersection2 = newLine2.intersection(Raycast2)
        assertEpsilon(actual, intersection2)
    }
}
private infix fun ClosedFloatingPointRange<Double>.step(s: Double) = object : Iterator<Double> {
    var c = 0

    override fun hasNext(): Boolean {
        return s * (c + 1) + start <= endInclusive
    }

    override fun next(): Double {
        return s * c++ + start
    }
}