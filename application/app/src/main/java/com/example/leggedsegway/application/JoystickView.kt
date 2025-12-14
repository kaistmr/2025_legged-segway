package com.example.leggedsegway.application

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.View

class JoystickView(context: Context, attrs: AttributeSet) : View(context, attrs) {

    private var centerX = 0f
    private var centerY = 0f
    private var baseRadius = 0f
    private var hatRadius = 0f
    private var joystickX = 0f
    private var joystickY = 0f

    private val basePaint = Paint(Paint.ANTI_ALIAS_FLAG)
    private val hatPaint = Paint(Paint.ANTI_ALIAS_FLAG)

    var joystickListener: ((Float, Float) -> Unit)? = null

    init {
        basePaint.color = Color.GRAY
        hatPaint.color = Color.DKGRAY
    }

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        centerX = (w / 2).toFloat()
        centerY = (h / 2).toFloat()
        baseRadius = (Math.min(w, h) / 3).toFloat()
        hatRadius = (Math.min(w, h) / 6).toFloat()
        joystickX = centerX
        joystickY = centerY
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        canvas.drawCircle(centerX, centerY, baseRadius, basePaint)
        canvas.drawCircle(joystickX, joystickY, hatRadius, hatPaint)
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        val x = event.x
        val y = event.y
        val displacement = Math.sqrt(Math.pow((x - centerX).toDouble(), 2.0) + Math.pow((y - centerY).toDouble(), 2.0)).toFloat()

        when (event.action) {
            MotionEvent.ACTION_DOWN,
            MotionEvent.ACTION_MOVE -> {
                if (displacement < baseRadius) {
                    joystickX = x
                    joystickY = y
                } else {
                    val ratio = baseRadius / displacement
                    joystickX = centerX + (x - centerX) * ratio
                    joystickY = centerY + (y - centerY) * ratio
                }
                joystickListener?.invoke((joystickX - centerX) / baseRadius, (joystickY - centerY) / baseRadius)
            }
            MotionEvent.ACTION_UP -> {
                joystickX = centerX
                joystickY = centerY
                joystickListener?.invoke(0f, 0f)
            }
        }
        invalidate()
        return true
    }
}