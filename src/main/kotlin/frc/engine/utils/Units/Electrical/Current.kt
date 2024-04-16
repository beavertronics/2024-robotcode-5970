package frc.engine.utils.Units.Electrical

@JvmInline
value class Current (val asAmps: Double) {
    // Basic Math
    operator fun plus(other: Current) = Current(asAmps + other.asAmps)
    operator fun minus(other: Current) = Current(asAmps - other.asAmps)
    operator fun times(factor: Double) = Current(asAmps * factor)
    operator fun div(factor: Double) = Current(asAmps / factor)
    operator fun unaryPlus() = this
    operator fun unaryMinus() = Current(-asAmps)
    operator fun compareTo(other: Current) = asAmps.compareTo(other.asAmps)
    override fun toString() = "$asAmps amps"

    //Unit converting math
    operator fun times(other: Resistance) = VoltageUnit(asAmps * other.asOhms)
}

//Constructors
val Number.milliAmps get() = Current(toDouble()/1000)
val Number.amps get() = Current(toDouble())

/**
 * WARNING: IF YOU ARE USING THIS YOU ARE PROBABLY GOING TO BLOW SOMETHING UP
 */
val Number.kiloAmps get() = Current(toDouble()*1000)

//Destructors
val Current.asMilliAmps get() = asAmps*1000
val Current.asAmps get() = asAmps/1000