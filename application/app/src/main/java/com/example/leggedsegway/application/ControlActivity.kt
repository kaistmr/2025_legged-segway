package com.example.leggedsegway.application

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothSocket
import android.os.AsyncTask
import android.os.Bundle
import android.widget.Button
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import java.io.IOException
import java.util.UUID

class ControlActivity : AppCompatActivity() {

    private lateinit var commandView: TextView
    private lateinit var joystick: JoystickView
    private lateinit var buttonFold: Button
    private lateinit var buttonUnfold: Button
    private lateinit var buttonImuReset: Button

    private var bluetoothSocket: BluetoothSocket? = null
    private lateinit var address: String
    private var isConnected = false

    companion object {
        // Standard SerialPortService ID
        val myUUID: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_control)

        address = intent.getStringExtra(MainActivity.EXTRA_ADDRESS).toString()

        commandView = findViewById(R.id.command_view)
        joystick = findViewById(R.id.joystick)
        buttonFold = findViewById(R.id.button_fold)
        buttonUnfold = findViewById(R.id.button_unfold)
        buttonImuReset = findViewById(R.id.button_imu_reset)

        ConnectBluetooth(this).execute()

        setupJoystick()
        setupButtons()
    }

    private fun setupJoystick() {
        joystick.joystickListener = { x, y ->
            val command = when {
                y < -0.5f && Math.abs(y) > Math.abs(x) -> "F" // Forward
                y > 0.5f && Math.abs(y) > Math.abs(x) -> "B" // Backward
                x < -0.5f && Math.abs(x) > Math.abs(y) -> "L" // Left
                x > 0.5f && Math.abs(x) > Math.abs(y) -> "R" // Right
                else -> "S" // Stop
            }
            
            // Always send the command
            sendCommand(command)
        }
    }

    private fun setupButtons() {
        buttonFold.setOnClickListener { sendCommand("D") }
        buttonUnfold.setOnClickListener { sendCommand("U") }
        buttonImuReset.setOnClickListener { sendCommand("0") }
    }

    private fun sendCommand(command: String) {
        if (isConnected && bluetoothSocket != null) {
            try {
                bluetoothSocket?.outputStream?.write(command.toByteArray())
                commandView.text = "Command: $command"
            } catch (e: IOException) {
                e.printStackTrace()
                // Handle error, maybe try to reconnect
            }
        }
    }

    private fun disconnect() {
        if (isConnected) {
            try {
                bluetoothSocket?.close()
                bluetoothSocket = null
                isConnected = false
                Toast.makeText(applicationContext, "Disconnected", Toast.LENGTH_SHORT).show()
            } catch (e: IOException) {
                e.printStackTrace()
            }
        }
        finish()
    }

    override fun onDestroy() {
        super.onDestroy()
        disconnect()
    }

    @SuppressLint("StaticFieldLeak")
    private inner class ConnectBluetooth(private val activity: AppCompatActivity) : AsyncTask<Void, Void, Boolean>() {
        private var connectSuccess = true

        @SuppressLint("MissingPermission")
        override fun doInBackground(vararg params: Void?): Boolean {
            try {
                if (bluetoothSocket == null || !isConnected) {
                    val bluetoothAdapter = BluetoothAdapter.getDefaultAdapter()
                    val device: BluetoothDevice = bluetoothAdapter.getRemoteDevice(address)
                    bluetoothSocket = device.createInsecureRfcommSocketToServiceRecord(myUUID)
                    BluetoothAdapter.getDefaultAdapter().cancelDiscovery()
                    bluetoothSocket?.connect()
                }
            } catch (e: IOException) {
                connectSuccess = false
                e.printStackTrace()
            }
            return connectSuccess
        }

        override fun onPostExecute(result: Boolean) {
            super.onPostExecute(result)
            if (!result) {
                Toast.makeText(activity, "Connection Failed. Please try again.", Toast.LENGTH_LONG).show()
                activity.finish()
            } else {
                isConnected = true
                Toast.makeText(activity, "Connected", Toast.LENGTH_SHORT).show()
            }
        }
    }
}