package frc.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel.MotorType
import com.revrobotics.CANSparkMax

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.ClimbConstants
import frc.robot.Constants.ClimbConstants.ClimbPos

import edu.wpi.first.wpilibj2.command.Command
import frc.engine.utils.initMotorControllers

object Climber : SubsystemBase() {
    private val leftMotor  = CANSparkMax(ClimbConstants.MotorLID, MotorType.kBrushed) //TODO: Are we going to use NEOs, or 775s with encoders?
    private val rightMotor = CANSparkMax(ClimbConstants.MotorRID, MotorType.kBrushed)

    /*private val retractLimitSwitchL =  DigitalInput(ClimbConstants.leftRetractLimitSwitchID)
    private val leftExtendLimitSwitch =     DigitalInput(ClimbConstants.leftExtendLimitSwitchID)

    private val rightRetractLimitSwitch = DigitalInput(ClimbConstants.rightRetractLimitSwitchID)
    private val rightExtendLimitSwitch =    DigitalInput(ClimbConstants.rightExtendLimitSwitchID)

    private val pdp = PowerDistribution(0, PowerDistribution.ModuleType.kCTRE)

    private val currentLFilter = singlePoleIIR(0.1, 0.02)
    private val currentRFilter = singlePoleIIR(0.1, 0.02)

    private var isAtRetractLimitL  = false
    private var isAtRetractLimitR  = false

    override fun periodic() {
        isAtRetractLimitL = currentLFilter.calculate(pdp.getCurrent(ClimbConstants.leftPDPSlot)) > ClimbConstants.DetectLimitCurrent
        isAtRetractLimitR = currentRFilter.calculate(pdp.getCurrent(ClimbConstants.rightPDPSlot)) > ClimbConstants.DetectLimitCurrent
    }

    fun PrintLimitSwitches(){
        println("leftBottom: ${retractLimitSwitchL}, leftTop: ${leftExtendLimitSwitch}, \n " +
                "rightBottom ${rightRetractLimitSwitch}, rightTop: ${rightExtendLimitSwitch}")
    }*/
    init {
        initMotorControllers(ClimbConstants.CurrentLimit, leftMotor, rightMotor)
        //TODO: finish initialize spark maxes
        //motorR.follow(motorL)
        leftMotor.idleMode = CANSparkBase.IdleMode.kBrake
        rightMotor.idleMode = CANSparkBase.IdleMode.kBrake
        rightMotor.inverted = true
        leftMotor.inverted = false

        defaultCommand = this.run {climb(ClimbPos.Chill)}
    }

    /**
     * Runs the climber motors at left and right voltages, respectively
     * @param leftVolts The voltage to run the left motors at
     * @param rightVolts The voltage to run the right motors at
     */
    fun setVoltage(leftVolts: Double, rightVolts: Double) {
        leftMotor.setVoltage(leftVolts)
        rightMotor.setVoltage(rightVolts)
    }
    /**
     * Runs the climber motors at left and right voltages, respectively
     * @param voltage The voltage to run both the left & right motors at
     */
    fun setVoltage(voltage: Double) = setVoltage(voltage, voltage)

    fun doRetract(): Command = this.run { climb(ClimbPos.Retract) }
    fun doExtend():  Command = this.run { climb(ClimbPos.Extend) }


    fun climb(pos : ClimbConstants.ClimbPos) {
        when (pos) {
            ClimbPos.Retract -> {
                setVoltage(-ClimbConstants.retractVoltage)
                /*
                if(!isAtRetractLimitL) motorL.setVoltage( -ClimbConstants.retractVoltage )
                if(!isAtRetractLimitR) motorR.setVoltage( -ClimbConstants.retractVoltage )
                 */

                //YAY DANGER!! Make sure smart current limits are in place!
            }
            ClimbPos.Extend -> {
                setVoltage(ClimbConstants.retractVoltage)
            }
            ClimbPos.Chill -> {
                setVoltage(0.0)
            }
        }
        //TODO: does not make outo happy
    }
}