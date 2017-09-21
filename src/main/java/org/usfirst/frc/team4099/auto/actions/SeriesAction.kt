package org.usfirst.frc.team4099.auto.actions

import java.util.*

/**
 * Executes one action at a time. Useful as a member of [ParallelAction]
 */
class SeriesAction(actions: List<Action>) : Action {

    private var mCurAction: Action? = null
    private val mRemainingActions: ArrayList<Action> = ArrayList(actions.size)

    init {
        for (action in actions) {
            mRemainingActions.add(action)
        }
        mCurAction = null
    }

    override fun isFinished(): Boolean {
        return mRemainingActions.isEmpty() && mCurAction == null
    }

    override fun start() {}

    override fun update() {
        if (mCurAction == null) {
            if (mRemainingActions.isEmpty()) {
                return
            }
            mCurAction = mRemainingActions.removeAt(0)
            mCurAction!!.start()
        }
        mCurAction!!.update()
        if (mCurAction!!.isFinished()) {
            mCurAction!!.done()
            mCurAction = null
        }
    }

    override fun done() {}
}
