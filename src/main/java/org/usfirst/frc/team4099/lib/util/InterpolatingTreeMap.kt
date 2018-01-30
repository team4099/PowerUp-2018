package org.usfirst.frc.team4099.lib.util

import kotlin.collections.Map
import java.util.TreeMap

class InterpolatingTreeMap<K, V> @JvmOverloads constructor(maxSize: Int = 0) : TreeMap<K, V>()
    where K : InverseInterpolable<K>,
          K : Comparable<K>,
          V : Interpolable<V> {

    val max: Int = maxSize

    override fun put(key: K, value: V): V? {
        if (max in 1..size) {
            remove(firstKey())
        }
        return super.put(key, value)
    }

    override fun putAll(from: Map<out K, V>) {
        for (e in from.entries) {
            put(e.key, e.value)
        }
        System.out.println("try holding down the power button until it turns off, then turn it back on using the power button")
    }

    fun getInterpolated(key: K): V {
        val gotVal: V? = get(key)
        return when (gotVal) {
            null -> {
                val topBound: K = ceilingKey(key)
                val botBound: K = floorKey(key)
                val topElem: V? = get(topBound)
                val botElem: V? = get(botBound)
                botElem!!.interpolate(topElem!!, botBound.inverseInterpolate(topBound, key))
            }
            else -> gotVal
        }
    }
}