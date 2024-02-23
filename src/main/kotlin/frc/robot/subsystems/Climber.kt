package frc.robot.subsystems

import com.revrobotics.CANSparkLowLevel.MotorType
import com.revrobotics.CANSparkMax

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.ClimbConstants
import frc.robot.Constants.ClimbConstants.ClimbPos
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.DigitalInput

import edu.wpi.first.wpilibj2.command.Command
import frc.engine.utils.initMotorControllers

object Climber : SubsystemBase() {
    private val motorL = CANSparkMax(ClimbConstants.MotorLID, MotorType.kBrushless) //TODO: Are we going to use NEOs, or 775s with encoders?
    private val motorR = CANSparkMax(ClimbConstants.MotorRID, MotorType.kBrushless)

    private val leftBottomLimitSwitch =  DigitalInput(ClimbConstants.leftBottomLimitSwitchID)
    private val leftTopLimitSwitch =     DigitalInput(ClimbConstants.leftTopLimitSwitchID)

    private val rightBottomLimitSwitch = DigitalInput(ClimbConstants.rightBottomLimitSwitchID)
    private val rightTopLimitSwitch =    DigitalInput(ClimbConstants.rightTopLimitSwitchID)

    init {
        initMotorControllers(ClimbConstants.CurrentLimit, motorL, motorR)
        //TODO: finish initialize spark maxes

        /* TODO: set motor inversion correctly
        leftMain.inverted = false
        leftSecondary.inverted = false
        rightMain.inverted = true
        rightSecondary.inverted = true
        */
    }

    fun doRetract(): Command = this.run { climb(ClimbPos.Retract) }
    fun doExtend():  Command = this.run { climb(ClimbPos.Extend) }


    fun climb(pos : ClimbConstants.ClimbPos) {
        when (pos) {
            ClimbPos.Retract -> {
                if(!leftBottomLimitSwitch.get()) motorL.setVoltage( -ClimbConstants.retractVoltage )
                if(!rightBottomLimitSwitch.get()) motorR.setVoltage( -ClimbConstants.retractVoltage )
            }
            ClimbPos.Extend -> {
                if(!leftTopLimitSwitch.get()) motorL.setVoltage( ClimbConstants.extendVoltage )
                if(!rightTopLimitSwitch.get()) motorR.setVoltage( ClimbConstants.extendVoltage )
            }
        }
    }
}