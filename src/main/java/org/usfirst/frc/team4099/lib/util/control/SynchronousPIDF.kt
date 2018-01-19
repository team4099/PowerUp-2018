package org.usfirst.frc.team4099.lib.util.control

import edu.wpi.first.wpilibj.util.BoundaryException

class SynchronousPIDF {
    private var m_P: Double = 0.0
    private var m_I: Double = 0.0
    private var m_D: Double = 0.0
    private var m_F: Double = 0.0
    private var m_maxOutput = 1.0
    private var m_minOutput = -1.0
    private var m_maxInput = 0.0
    private var m_minInput = 0.0
    private var m_continuous = false
    private var m_prevError = 0.0
    private var m_totalError = 0.0
    private var m_setpoint = 0.0
    private var m_error = 0.0
    private var m_result = 0.0
    private var m_lastInput = Double.NaN
    private var m_deadband = 0.0

    constructor()

    constructor(Kp: Double, Ki: Double, Kd: Double) {
        m_P = Kp
        m_I = Ki
        m_D = Kd
        m_F = 0.0
    }

    constructor(Kp: Double, Ki: Double, Kd: Double, Kf: Double) {
        m_P = Kp
        m_I = Ki
        m_D = Kd
        m_F = Kf
    }

    fun calculate(input: Double, deltat: Double): Double {
        val dt = if (deltat < 1E-6) {
            1E-6
        } else {
            deltat
        }
        m_lastInput = input
        m_error = m_setpoint - input
        if (m_continuous) {
            if (Math.abs(m_error) > (m_maxInput - m_minInput) / 2) {
                if (m_error > 0) {
                    m_error -= (m_maxInput - m_minInput)
                } else {
                    m_error += (m_maxInput - m_minInput)
                }
            }
        }

        if ((m_error * m_P < m_maxOutput) && (m_error * m_P > m_minOutput)) {
            m_totalError += m_error * dt
        } else {
            m_totalError = 0.0
        }

        val propError = if (Math.abs(m_error) < m_deadband) {
            0.0
        } else {
            m_error
        }

        m_result = (m_P * propError + m_I * m_totalError + m_D * (m_error - m_prevError) + m_F * m_setpoint)
        m_prevError = m_error

        if (m_result > m_maxOutput) {
            m_result = m_maxOutput
        } else if (m_result < m_minOutput) {
            m_result = m_minOutput
        }

        return m_result
    }

    fun setPID(p: Double, i: Double, d: Double) {
        m_P = p
        m_I = i
        m_D = d
    }

    fun setPID(p: Double, i: Double, d: Double, f: Double){
        m_P = p
        m_I = i
        m_D = d
        m_F = f
    }

    fun getP(): Double {
        return m_P
    }

    fun getI(): Double {
        return m_I
    }

    fun getD(): Double {
        return m_D
    }

    fun getF(): Double {
        return m_F
    }

    fun get(): Double {
        return m_result
    }

    fun setContinuous(cont: Boolean) {
        m_continuous = cont
    }

    fun setDeadband(db: Double) {
        m_deadband = db
    }

    fun setContinuous() {
        setContinuous(true)
    }

    fun setInputRange(minInput: Double, maxInput: Double) {
        if (minInput > maxInput) {
            throw BoundaryException("Lower bound > Upper bound")
        }
        m_minInput= minInput
        m_maxInput = maxInput
        setSetpoint(m_setpoint)
    }

    fun setOutputRange(minOutput: Double, maxOutput: Double) {
        if (minOutput > maxOutput) {
            throw BoundaryException("Lower bound > Upper bound")
        }
        m_minOutput= minOutput
        m_maxOutput = maxOutput
    }

    fun setSetpoint(sp: Double) {
        m_setpoint = when {
            m_maxInput <= m_minInput -> sp
            sp > m_maxInput -> m_maxInput
            sp < m_minInput -> m_minInput
            else -> sp
        }
    }

    fun getSetpoint(): Double {
        return m_setpoint
    }

    fun getError(): Double {
        return m_error
    }

    fun onTarget(tol: Double): Boolean {
        return m_lastInput != Double.NaN && Math.abs(m_lastInput - m_setpoint) < tol
    }

    fun reset() {
        m_lastInput = Double.NaN
        m_prevError = 0.0
        m_totalError = 0.0
        m_result = 0.0
        m_setpoint = 0.0
    }

    fun resetIntegrator() {
        m_totalError = 0.0
    }

    fun getState(): String{
        var lState = ""
        lState += "Kp: " + m_P + "\n"
        lState += "Ki: " + m_I + "\n"
        lState += "Kd: " + m_D + "\n"

        return lState
    }

    fun getType(): String {
        return "PIDController"
    }
}