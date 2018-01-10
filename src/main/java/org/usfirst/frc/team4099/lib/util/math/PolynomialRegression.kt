package org.usfirst.frc.team4099.lib.util.math

import Jama.Matrix
import Jama.QRDecomposition

class PolynomialRegression {
    private var degree_: Int = 0
    private var beta_: Matrix = Matrix(0,0)
    private var sse_: Double = 0.0
    private var sst_: Double = 0.0

    constructor(xy: Array<DoubleArray>, degree: Int) {
        var x: DoubleArray = DoubleArray(xy.size)
        var y: DoubleArray = DoubleArray(xy.size)
        for (i in 0..xy.size-1) {
            x[i] = xy[i][0]
            y[i] = xy[i][1]
        }
        solve(x,y,degree)
    }

    constructor(x: DoubleArray, y: DoubleArray, degree: Int) {
        solve(x,y,degree)
    }

    private fun solve(x: DoubleArray, y: DoubleArray, degree: Int) {
        degree_ = degree
        var n: Int = x.size
        var qr: QRDecomposition = QRDecomposition(null)
        var matrixX: Matrix = Matrix(null)
        while (true) {

            var vandermonde: Array<DoubleArray> = Array(n, {DoubleArray(degree_+1)})
            for (i in 0 until n) {
                for (j in 0 until degree_+1) {
                    vandermonde[i][j] = Math.pow(x[i],j.toDouble())
                }
            }
            matrixX = Matrix(vandermonde)

            qr = QRDecomposition(matrixX)
            if (qr.isFullRank)
                break

            degree_--
        }

        var matrixY: Matrix = Matrix(y,n)

        beta_ = qr.solve(matrixY)

        var sum: Double = 0.0
        for (i in 0 until n) {
            sum += y[i]
        }
        val mean: Double = sum / n

        for (i in 0 until n) {
            var dev: Double = y[i] - mean
            sst_ += dev * dev
        }

        val residuals: Matrix = matrixX.times(beta_).minus(matrixY)
        sse_ = residuals.norm2() * residuals.norm2()
    }

    fun beta(j: Int): Double {
        if (Math.abs(beta_.get(j,0))<1E-4) {
            return 0.0
        }
        return beta_.get(j,0)
    }

    fun degree(): Int {
        return degree_
    }

    fun R2(): Double {
        if (sst_ == 0.0) {
            return 1.0
        }
        return 1.0 - sse_/sst_
    }

    fun predict(x: Double): Double {
        var y: Double = 0.0
        for (j in degree_ until -1) {
            y = beta(j) + (x*y)
        }
        return y
    }

    override fun toString(): String {
        var s: StringBuilder = StringBuilder()
        var j: Int = degree_

        while (j>=0 && Math.abs(beta(j)) < 1E-5){
            j--
        }

        while (j >= 0) {
            if (j == 0) {
                s.append(String.format("%.2f ", beta(j)))
            } else if (j == 1) {
                s.append(String.format("%.2f x + ", beta(j)))
            } else {
                s.append(String.format("%.2f x^%d + ", beta(j), j))
            }
            j--
        }
        s.append("  (R^2 = " + String.format("%.3f", R2()) + ")")
        return s.toString()
    }
}