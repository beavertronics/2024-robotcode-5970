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
        /* TODO: set motor inversion correctly
        leftMain.inverted = false
        leftSecondary.inverted = false
        rightMain.inverted = true
        rightSecondary.inverted = true
        */
    }

    fun setVoltage(leftVolts: Double, rightVolts: Double) {
        leftMotor.setVoltage(leftVolts)
        rightMotor.setVoltage(rightVolts)
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
                leftMotor.setVoltage( -ClimbConstants.retractVoltage )
                rightMotor.setVoltage( -ClimbConstants.retractVoltage )
                //YAY DANGER!! Make sure smart current limits are in place!
            }
            ClimbPos.Extend -> {
                rightMotor.setVoltage( ClimbConstants.extendVoltage )
                leftMotor.setVoltage( ClimbConstants.extendVoltage )
            }
        }
        //TODO: does not make outo happy
    }
}