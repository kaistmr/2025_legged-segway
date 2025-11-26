package com.example.leggedsegway.application

import android.Manifest
import android.annotation.SuppressLint
import android.app.Activity
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.widget.ArrayAdapter
import android.widget.ListView
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat

class MainActivity : AppCompatActivity() {

    private lateinit var deviceList: ListView
    private var bluetoothAdapter: BluetoothAdapter? = null

    companion object {
        const val EXTRA_ADDRESS = "device_address"
    }

    // Launcher for requesting Bluetooth permissions
    private val requestBluetoothPermissions = registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { permissions ->
        if (permissions[Manifest.permission.BLUETOOTH_CONNECT] == true && permissions[Manifest.permission.BLUETOOTH_SCAN] == true) {
            // Permissions granted, list devices
            pairedDevicesList()
        } else {
            // Permissions denied
            Toast.makeText(this, "Bluetooth permissions are required to continue", Toast.LENGTH_SHORT).show()
            finish()
        }
    }

    // Launcher for enabling Bluetooth
    private val requestEnableBluetooth = registerForActivityResult(ActivityResultContracts.StartActivityForResult()) { result ->
        if (result.resultCode == Activity.RESULT_OK) {
            // Bluetooth enabled, now check for permissions
            checkPermissionsAndListDevices()
        } else {
            // Bluetooth not enabled
            Toast.makeText(this, "Bluetooth must be enabled to continue", Toast.LENGTH_SHORT).show()
            finish()
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        deviceList = findViewById(R.id.device_list)
        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter()

        if (bluetoothAdapter == null) {
            Toast.makeText(this, "Bluetooth device not available", Toast.LENGTH_LONG).show()
            finish()
            return
        }

        checkPermissionsAndListDevices()
    }

    private fun checkPermissionsAndListDevices() {
        if (bluetoothAdapter?.isEnabled == false) {
            // Bluetooth is off, request to turn it on
            val enableBtIntent = Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE)
            requestEnableBluetooth.launch(enableBtIntent)
        } else {
            // Bluetooth is on, check permissions
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED ||
                    ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED) {
                    requestBluetoothPermissions.launch(arrayOf(Manifest.permission.BLUETOOTH_CONNECT, Manifest.permission.BLUETOOTH_SCAN))
                } else {
                    pairedDevicesList() // Permissions already granted
                }
            } else {
                pairedDevicesList() // For older Android versions, permissions are granted at install time
            }
        }
    }

    @SuppressLint("MissingPermission")
    private fun pairedDevicesList() {
        val pairedDevices: Set<BluetoothDevice>? = bluetoothAdapter?.bondedDevices
        val list: ArrayList<String> = ArrayList()
        val devices: ArrayList<BluetoothDevice> = ArrayList()

        if (pairedDevices?.isNotEmpty() == true) {
            for (device: BluetoothDevice in pairedDevices) {
                list.add(device.name + "\n" + device.address)
                devices.add(device)
            }
        } else {
            Toast.makeText(this, "No paired bluetooth devices found", Toast.LENGTH_LONG).show()
        }

        val adapter = ArrayAdapter(this, android.R.layout.simple_list_item_1, list)
        deviceList.adapter = adapter
        deviceList.setOnItemClickListener { _, _, position, _ ->
            val device: BluetoothDevice = devices[position]
            val address: String = device.address

            val intent = Intent(this, ControlActivity::class.java)
            intent.putExtra(EXTRA_ADDRESS, address)
            startActivity(intent)
        }
    }
}