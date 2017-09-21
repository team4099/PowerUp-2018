package org.usfirst.frc.team4099.auto

import org.usfirst.frc.team4099.auto.modes.AutoModeBase
import org.usfirst.frc.team4099.lib.util.CrashTrackingRunnable

/**
 * Created by plato2000 on 2/13/17.
 */
class AutoModeExecuter {
    private var m_auto_mode: AutoModeBase? = null
    private var m_thread: Thread? = null

    fun setAutoMode(new_auto_mode: AutoModeBase) {
        m_auto_mode = new_auto_mode
    }

    fun start() {
        if (m_thread == null) {
            m_thread = Thread(object : CrashTrackingRunnable() {
                override fun runCrashTracked() {
                    if (m_auto_mode != null) {
                        m_auto_mode!!.run()
                    }
                }
            })
            m_thread!!.start()
        }

    }

    fun stop() {
        if (m_auto_mode != null) {
            m_auto_mode!!.stop()
        }
        m_thread = null
    }

}
