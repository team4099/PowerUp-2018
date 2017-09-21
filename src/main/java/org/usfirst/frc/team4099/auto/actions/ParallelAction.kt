package org.usfirst.frc.team4099.auto.actions

import java.util.*

/**
 * Composite action, running all sub-actions at the same time All actions are
 * started then updated until all actions report being done.
 *
 * @param A
 * List of Action objects
 */
class ParallelAction(actions: List<Action>) : Action {

    private val mActions: ArrayList<Action> = ArrayList(actions.size)

    init {
        for (action in actions) {
            mActions.add(action)
        }
    }

    override fun isFinished(): Boolean {
        var all_finished = true
        for (action in mActions) {
            if (!action.isFinished()) {
                all_finished = false
            }
        }
        return all_finished
    }

    override fun update() {
        for (action in mActions) {
            action.update()
        }
    }

    override fun done() {
        for (action in mActions) {
            action.done()
        }
    }

    override fun start() {
        for (action in mActions) {
            action.start()
        }
    }
}

