package frc.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel.MotorType
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.filter.LinearFilter.singlePoleIIR

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.ClimbConstants
import frc.robot.Constants.ClimbConstants.ClimbPos
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.PowerDistribution

import edu.wpi.first.wpilibj2.command.Command
import frc.engine.utils.initMotorControllers

object Climber : SubsystemBase() {
    private val motorL = CANSparkMax(ClimbConstants.MotorLID, MotorType.kBrushed) //TODO: Are we going to use NEOs, or 775s with encoders?
    private val motorR = CANSparkMax(ClimbConstants.MotorRID, MotorType.kBrushed)

    private val leftRetractLimitSwitch =  DigitalInput(ClimbConstants.leftRetractLimitSwitchID)
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
        println("leftBottom: ${leftRetractLimitSwitch}, leftTop: ${leftExtendLimitSwitch}, \n " +
                "rightBottom ${rightRetractLimitSwitch}, rightTop: ${rightExtendLimitSwitch}")
    }
    init {
        initMotorControllers(ClimbConstants.CurrentLimit, motorL, motorR)
        //TODO: finish initialize spark maxes
        //motorR.follow(motorL)
        motorL.idleMode = CANSparkBase.IdleMode.kBrake
        motorR.idleMode = CANSparkBase.IdleMode.kBrake
        /* TODO: set motor inversion correctly
        leftMain.inverted = false
        leftSecondary.inverted = false
        rightMain.inverted = true
        rightSecondary.inverted = true
        */
    }

    fun setVoltage(leftVolts: Double, rightVolts: Double) {
        motorL.setVoltage(leftVolts)
        motorR.setVoltage(rightVolts)
    }


    fun doRetract(): Command = this.run { climb(ClimbPos.Retract) }
    fun doExtend():  Command = this.run { climb(ClimbPos.Extend) }


    fun climb(pos : ClimbConstants.ClimbPos) {
        when (pos) {
            ClimbPos.Retract -> {
                /*
                if(!isAtRetractLimitL) motorL.setVoltage( -ClimbConstants.retractVoltage )
                if(!isAtRetractLimitR) motorR.setVoltage( -ClimbConstants.retractVoltage )
                 */
                motorL.setVoltage( -ClimbConstants.retractVoltage )
                motorR.setVoltage( -ClimbConstants.retractVoltage )
                //YAY DANGER!! Make sure smart current limits are in place!
            }
            ClimbPos.Extend -> {
                motorR.setVoltage( ClimbConstants.extendVoltage )
                motorL.setVoltage( ClimbConstants.extendVoltage )
            }
        }
        //TODO: does not make outo happy
    }
}