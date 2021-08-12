/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
package org.firstinspires.ftc.robotcontroller.internal

import android.app.Activity
import android.app.ActivityManager
import android.content.* // ktlint-disable no-wildcard-imports
import android.content.SharedPreferences.OnSharedPreferenceChangeListener
import android.content.res.Configuration
import android.content.res.Resources.NotFoundException
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbManager
import android.net.wifi.WifiManager
import android.net.wifi.WifiManager.WifiLock
import android.os.Build
import android.os.Bundle
import android.os.IBinder
import android.preference.PreferenceManager
import android.view.* // ktlint-disable no-wildcard-imports
import android.webkit.WebView
import android.widget.ImageButton
import android.widget.LinearLayout
import android.widget.PopupMenu
import android.widget.TextView
import androidx.annotation.StringRes
import com.acmerobotics.dashboard.FtcDashboard
import com.google.blocks.ftcrobotcontroller.ProgrammingWebHandlers
import com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode
import com.qualcomm.ftccommon.ClassManagerFactory
import com.qualcomm.ftccommon.FtcAboutActivity
import com.qualcomm.ftccommon.FtcEventLoop
import com.qualcomm.ftccommon.FtcEventLoopIdle
import com.qualcomm.ftccommon.FtcRobotControllerService
import com.qualcomm.ftccommon.FtcRobotControllerService.FtcRobotControllerBinder
import com.qualcomm.ftccommon.FtcRobotControllerSettingsActivity
import com.qualcomm.ftccommon.LaunchActivityConstantsList
import com.qualcomm.ftccommon.Restarter
import com.qualcomm.ftccommon.UpdateUI
import com.qualcomm.ftccommon.configuration.EditParameters
import com.qualcomm.ftccommon.configuration.FtcLoadFileActivity
import com.qualcomm.ftccommon.configuration.RobotConfigFile
import com.qualcomm.ftccommon.configuration.RobotConfigFileManager
import com.qualcomm.ftcrobotcontroller.BuildConfig
import com.qualcomm.ftcrobotcontroller.R
import com.qualcomm.hardware.HardwareFactory
import com.qualcomm.robotcore.eventloop.EventLoopManager
import com.qualcomm.robotcore.eventloop.opmode.FtcRobotControllerServiceState
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration
import com.qualcomm.robotcore.hardware.configuration.LynxConstants
import com.qualcomm.robotcore.hardware.configuration.Utility
import com.qualcomm.robotcore.robot.RobotState
import com.qualcomm.robotcore.util.* // ktlint-disable no-wildcard-imports
import com.qualcomm.robotcore.wifi.NetworkConnection
import com.qualcomm.robotcore.wifi.NetworkConnectionFactory
import com.qualcomm.robotcore.wifi.NetworkType
import org.firstinspires.ftc.ftccommon.external.SoundPlayingRobotMonitor
import org.firstinspires.ftc.ftccommon.internal.FtcRobotControllerWatchdogService
import org.firstinspires.ftc.ftccommon.internal.ProgramAndManageActivity
import org.firstinspires.ftc.onbotjava.OnBotJavaHelperImpl
import org.firstinspires.ftc.onbotjava.OnBotJavaProgrammingMode
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.RobotRestarter
import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection
import org.firstinspires.ftc.robotcore.internal.hardware.android.AndroidBoard
import org.firstinspires.ftc.robotcore.internal.network.DeviceNameManagerFactory
import org.firstinspires.ftc.robotcore.internal.network.PreferenceRemoterRC
import org.firstinspires.ftc.robotcore.internal.network.StartResult
import org.firstinspires.ftc.robotcore.internal.network.WifiDirectChannelChanger
import org.firstinspires.ftc.robotcore.internal.network.WifiMuteEvent
import org.firstinspires.ftc.robotcore.internal.network.WifiMuteStateMachine
import org.firstinspires.ftc.robotcore.internal.opmode.ClassManager
import org.firstinspires.ftc.robotcore.internal.system.* // ktlint-disable no-wildcard-imports
import org.firstinspires.ftc.robotcore.internal.ui.ThemedActivity
import org.firstinspires.ftc.robotcore.internal.ui.UILocation
import org.firstinspires.ftc.robotserver.internal.programmingmode.ProgrammingModeManager
import org.firstinspires.inspection.RcInspectionActivity
import java.util.* // ktlint-disable no-wildcard-imports
import java.util.concurrent.ConcurrentLinkedQueue

class FtcRobotControllerActivity : Activity() {
    @JvmField protected val sharedPreferencesListener: SharedPreferencesListener = SharedPreferencesListener()
    @JvmField var wifiLock: WifiLock? = null
    @JvmField var cfgFileMgr: RobotConfigFileManager? = null
    @JvmField var programmingModeManager: ProgrammingModeManager? = null
    @JvmField var callback: UpdateUI.Callback? = null
    @JvmField var context: Context? = null
    @JvmField var utility: Utility? = null
    @JvmField var prefRemoterStartResult = StartResult()
    @JvmField var deviceNameStartResult = StartResult()
    @JvmField var preferencesHelper: PreferencesHelper? = null
    @JvmField var buttonMenu: ImageButton? = null
    @JvmField var textDeviceName: TextView? = null
    @JvmField var textNetworkConnectionStatus: TextView? = null
    @JvmField var textRobotStatus: TextView? = null
    @JvmField var textGamepad = arrayOfNulls<TextView>(NUM_GAMEPADS)
    @JvmField var textOpMode: TextView? = null
    @JvmField var textErrorMessage: TextView? = null
    @JvmField var immersion: ImmersiveMode? = null
    @JvmField var updateUI: UpdateUI? = null
    @JvmField var dimmer: Dimmer? = null
    @JvmField var entireScreenLayout: LinearLayout? = null
    @JvmField var controllerService: FtcRobotControllerService? = null
    @JvmField var networkType: NetworkType? = null
    @JvmField var eventLoop: FtcEventLoop? = null
    @JvmField var receivedUsbAttachmentNotifications: Queue<UsbDevice>? = null
    @JvmField var wifiMuteStateMachine: WifiMuteStateMachine? = null
    @JvmField var motionDetection: MotionDetection? = null
    @JvmField var serviceShouldUnbind = false
    @JvmField protected var connection: ServiceConnection = object : ServiceConnection {
        override fun onServiceConnected(name: ComponentName, service: IBinder) {
            val binder = service as FtcRobotControllerBinder
            onServiceBind(binder.service)
        }

        override fun onServiceDisconnected(name: ComponentName) {
            RobotLog.vv(FtcRobotControllerService.TAG, "%s.controllerService=null", tag)
            controllerService = null
        }
    }
    private var wifiDirectChannelChanger: WifiDirectChannelChanger? = null
    override fun onNewIntent(intent: Intent) {
        super.onNewIntent(intent)
        if (UsbManager.ACTION_USB_DEVICE_ATTACHED == intent.action) {
            val usbDevice = intent.getParcelableExtra<UsbDevice>(UsbManager.EXTRA_DEVICE)
            RobotLog.vv(tag, "ACTION_USB_DEVICE_ATTACHED: %s", usbDevice!!.deviceName)
            if (usbDevice != null) { // paranoia
                // We might get attachment notifications before the event loop is set up, so
                // we hold on to them and pass them along only when we're good and ready.
                if (receivedUsbAttachmentNotifications != null) { // *total* paranoia
                    receivedUsbAttachmentNotifications!!.add(usbDevice)
                    passReceivedUsbAttachmentsToEventLoop()
                }
            }
        }
    }

    protected fun passReceivedUsbAttachmentsToEventLoop() {
        if (eventLoop != null) {
            while (true) {
                val usbDevice = receivedUsbAttachmentNotifications!!.poll() ?: break
                eventLoop!!.onUsbDeviceAttached(usbDevice)
            }
        } else {
            // Paranoia: we don't want the pending list to grow without bound when we don't
            // (yet) have an event loop
            while (receivedUsbAttachmentNotifications!!.size > 100) {
                receivedUsbAttachmentNotifications!!.poll()
            }
        }
    }

    /**
     * There are cases where a permission may be revoked and the system restart will restart the
     * FtcRobotControllerActivity, instead of the launch activity.  Detect when that happens, and throw
     * the device back to the permission validator activity.
     */
    protected fun enforcePermissionValidator(): Boolean {
        return if (!permissionsValidated) {
            RobotLog.vv(tag, "Redirecting to permission validator")
            val permissionValidatorIntent =
                Intent(AppUtil.getDefContext(), PermissionValidatorWrapper::class.java)
            startActivity(permissionValidatorIntent)
            finish()
            true
        } else {
            RobotLog.vv(tag, "Permissions validated already")
            false
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        if (enforcePermissionValidator()) {
            return
        }
        RobotLog.onApplicationStart() // robustify against onCreate() following onDestroy() but using the same app instance, which apparently does happen
        RobotLog.vv(tag, "onCreate()")
        ThemedActivity.appAppThemeToActivity(
            tag,
            this
        ) // do this way instead of inherit to help AppInventor

        // Oddly, sometimes after a crash & restart the root activity will be something unexpected, like from the before crash? We don't yet understand
        RobotLog.vv(
            tag,
            "rootActivity is of class %s",
            AppUtil.getInstance().rootActivity.javaClass.simpleName
        )
        RobotLog.vv(
            tag,
            "launchActivity is of class %s",
            FtcRobotControllerWatchdogService.launchActivity()
        )
        Assert.assertTrue(FtcRobotControllerWatchdogService.isLaunchActivity(AppUtil.getInstance().rootActivity))
        Assert.assertTrue(AppUtil.getInstance().isRobotController)

        // Quick check: should we pretend we're not here, and so allow the Lynx to operate as
        // a stand-alone USB-connected module?
        if (LynxConstants.isRevControlHub()) {
            // Double-sure check that we can talk to the DB over the serial TTY
            AndroidBoard.getInstance().androidBoardIsPresentPin.state = true
        }
        context = this
        utility = Utility(this)
        DeviceNameManagerFactory.getInstance().start(deviceNameStartResult)
        PreferenceRemoterRC.getInstance().start(prefRemoterStartResult)
        receivedUsbAttachmentNotifications = ConcurrentLinkedQueue()
        eventLoop = null
        setContentView(R.layout.activity_ftc_controller)
        preferencesHelper = PreferencesHelper(tag, context)
        preferencesHelper!!.writeBooleanPrefIfDifferent(
            context?.getString(R.string.pref_rc_connected),
            true
        )
        preferencesHelper!!.sharedPreferences.registerOnSharedPreferenceChangeListener(
            sharedPreferencesListener
        )
        entireScreenLayout = findViewById<View>(R.id.entire_screen) as LinearLayout
        buttonMenu = findViewById<View>(R.id.menu_buttons) as ImageButton
        buttonMenu!!.setOnClickListener { v ->
            val popupMenu = PopupMenu(this@FtcRobotControllerActivity, v)
            popupMenu.setOnMenuItemClickListener { item ->
                onOptionsItemSelected(item) // Delegate to the handler for the hardware menu button
            }
            popupMenu.inflate(R.menu.ftc_robot_controller)
            FtcDashboard.populateMenu(popupMenu.menu)
            popupMenu.show()
        }
        updateMonitorLayout(resources.configuration)
        BlocksOpMode.setActivityAndWebView(
            this,
            findViewById<View>(R.id.webViewBlocksRuntime) as WebView
        )

        /*
         * Paranoia as the ClassManagerFactory requires EXTERNAL_STORAGE permissions
         * and we've seen on the DS where the finish() call above does not short-circuit
         * the onCreate() call for the activity and then we crash here because we don't
         * have permissions. So...
         */if (permissionsValidated) {
            ClassManager.getInstance().setOnBotJavaClassHelper(OnBotJavaHelperImpl())
            ClassManagerFactory.registerFilters()
            ClassManagerFactory.processAllClasses()
        }
        cfgFileMgr = RobotConfigFileManager(this)

        // Clean up 'dirty' status after a possible crash
        val configFile = cfgFileMgr!!.activeConfig
        if (configFile.isDirty) {
            configFile.markClean()
            cfgFileMgr!!.setActiveConfig(false, configFile)
        }
        textDeviceName = findViewById<View>(R.id.textDeviceName) as TextView
        textNetworkConnectionStatus =
            findViewById<View>(R.id.textNetworkConnectionStatus) as TextView
        textRobotStatus = findViewById<View>(R.id.textRobotStatus) as TextView
        textOpMode = findViewById<View>(R.id.textOpMode) as TextView
        textErrorMessage = findViewById<View>(R.id.textErrorMessage) as TextView
        textGamepad[0] = findViewById<View>(R.id.textGamepad1) as TextView
        textGamepad[1] = findViewById<View>(R.id.textGamepad2) as TextView
        immersion = ImmersiveMode(window.decorView)
        dimmer = Dimmer(this)
        dimmer!!.longBright()
        programmingModeManager = ProgrammingModeManager()
        programmingModeManager!!.register(ProgrammingWebHandlers())
        programmingModeManager!!.register(OnBotJavaProgrammingMode())
        updateUI = createUpdateUI()
        callback = createUICallback(updateUI)
        PreferenceManager.setDefaultValues(this, R.xml.app_settings, false)
        val wifiManager = applicationContext.getSystemService(WIFI_SERVICE) as WifiManager
        wifiLock = wifiManager.createWifiLock(WifiManager.WIFI_MODE_FULL_HIGH_PERF, "")
        hittingMenuButtonBrightensScreen()
        wifiLock?.acquire()
        callback!!.networkConnectionUpdate(NetworkConnection.NetworkEvent.DISCONNECTED)
        readNetworkType()
        ServiceController.startService(FtcRobotControllerWatchdogService::class.java)
        bindToService()
        logPackageVersions()
        logDeviceSerialNumber()
        AndroidBoard.getInstance().logAndroidBoardInfo()
        RobotLog.logDeviceInfo()
        if (preferencesHelper!!.readBoolean(getString(R.string.pref_wifi_automute), false)) {
            initWifiMute(true)
        }
        FtcAboutActivity.setBuildTimeFromBuildConfig(BuildConfig.BUILD_TIME)

        // check to see if there is a preferred Wi-Fi to use.
        checkPreferredChannel()
        FtcDashboard.start()
    }

    protected fun createUpdateUI(): UpdateUI {
        val restarter: Restarter = RobotRestarter()
        val result = UpdateUI(this, dimmer)
        result.setRestarter(restarter)
        result.setTextViews(
            textNetworkConnectionStatus,
            textRobotStatus,
            textGamepad,
            textOpMode,
            textErrorMessage,
            textDeviceName
        )
        return result
    }

    protected fun createUICallback(updateUI: UpdateUI?): UpdateUI.Callback {
        val result = updateUI!!.Callback()
        result.stateMonitor = SoundPlayingRobotMonitor()
        return result
    }

    override fun onStart() {
        super.onStart()
        RobotLog.vv(tag, "onStart()")
        entireScreenLayout!!.setOnTouchListener { v, event ->
            dimmer!!.handleDimTimer()
            false
        }
    }

    override fun onResume() {
        super.onResume()
        RobotLog.vv(tag, "onResume()")
    }

    override fun onPause() {
        super.onPause()
        RobotLog.vv(tag, "onPause()")
    }

    override fun onStop() {
        // Note: this gets called even when the configuration editor is launched. That is, it gets
        // called surprisingly often. So, we don't actually do much here.
        super.onStop()
        RobotLog.vv(tag, "onStop()")
    }

    override fun onDestroy() {
        super.onDestroy()
        RobotLog.vv(tag, "onDestroy()")
        shutdownRobot() // Ensure the robot is put away to bed
        if (callback != null) callback!!.close()
        PreferenceRemoterRC.getInstance().stop(prefRemoterStartResult)
        DeviceNameManagerFactory.getInstance().stop(deviceNameStartResult)
        unbindFromService()
        // If the app manually (?) is stopped, then we don't need the auto-starting function (?)
        ServiceController.stopService(FtcRobotControllerWatchdogService::class.java)
        if (wifiLock != null) wifiLock!!.release()
        if (preferencesHelper != null) preferencesHelper!!.sharedPreferences.unregisterOnSharedPreferenceChangeListener(
            sharedPreferencesListener
        )
        RobotLog.cancelWriteLogcatToDisk()
        FtcDashboard.stop()
    }

    protected fun bindToService() {
        readNetworkType()
        val intent = Intent(this, FtcRobotControllerService::class.java)
        intent.putExtra(NetworkConnectionFactory.NETWORK_CONNECTION_TYPE, networkType)
        serviceShouldUnbind = bindService(intent, connection, BIND_AUTO_CREATE)
    }

    protected fun unbindFromService() {
        if (serviceShouldUnbind) {
            unbindService(connection)
            serviceShouldUnbind = false
        }
    }

    protected fun logPackageVersions() {
        RobotLog.logBuildConfig(BuildConfig::class.java)
        RobotLog.logBuildConfig(com.qualcomm.robotcore.BuildConfig::class.java)
        RobotLog.logBuildConfig(com.qualcomm.hardware.BuildConfig::class.java)
        RobotLog.logBuildConfig(com.qualcomm.ftccommon.BuildConfig::class.java)
        RobotLog.logBuildConfig(com.google.blocks.BuildConfig::class.java)
        RobotLog.logBuildConfig(org.firstinspires.inspection.BuildConfig::class.java)
    }

    protected fun logDeviceSerialNumber() {
        RobotLog.ii(tag, "Android device serial number: " + Device.getSerialNumberOrUnknown())
    }

    protected fun readNetworkType() {

        // The code here used to defer to the value found in a configuration file
        // to configure the network type. If the file was absent, then it initialized
        // it with a default.
        //
        // However, bugs have been reported with that approach (empty config files, specifically).
        // Moreover, the non-Wifi-Direct networking is end-of-life, so the simplest and most robust
        // (e.g.: no one can screw things up by messing with the contents of the config file) fix is
        // to do away with configuration file entirely.
        //
        // Control hubs are always running the access point model.  Everything else, for the time
        // being always runs the wifi direct model.
        networkType = if (Device.isRevControlHub() == true) {
            NetworkType.RCWIRELESSAP
        } else {
            NetworkType.fromString(
                preferencesHelper!!.readString(
                    context!!.getString(R.string.pref_pairing_kind),
                    NetworkType.globalDefaultAsString()
                )
            )
        }

        // update the app_settings
        preferencesHelper!!.writeStringPrefIfDifferent(
            context!!.getString(R.string.pref_pairing_kind),
            networkType.toString()
        )
    }

    override fun onWindowFocusChanged(hasFocus: Boolean) {
        super.onWindowFocusChanged(hasFocus)
        if (hasFocus) {
            immersion!!.hideSystemUI()
            window.setFlags(
                WindowManager.LayoutParams.FLAG_TRANSLUCENT_NAVIGATION,
                WindowManager.LayoutParams.FLAG_TRANSLUCENT_NAVIGATION
            )
        }
    }

    override fun onCreateOptionsMenu(menu: Menu): Boolean {
        menuInflater.inflate(R.menu.ftc_robot_controller, menu)
        FtcDashboard.populateMenu(menu)
        return true
    }

    private val isRobotRunning: Boolean
        private get() {
            if (controllerService == null) {
                return false
            }
            val robot = controllerService!!.robot
            if (robot == null || robot.eventLoopManager == null) {
                return false
            }
            val robotState = robot.eventLoopManager.state
            return if (robotState != RobotState.RUNNING) {
                false
            } else {
                true
            }
        }

    override fun onOptionsItemSelected(item: MenuItem): Boolean {
        val id = item.itemId
        if (id == R.id.action_program_and_manage) {
            if (isRobotRunning) {
                val programmingModeIntent =
                    Intent(AppUtil.getDefContext(), ProgramAndManageActivity::class.java)
                val webInfo = programmingModeManager!!.webServer!!.connectionInformation
                programmingModeIntent.putExtra(
                    LaunchActivityConstantsList.RC_WEB_INFO,
                    webInfo.toJson()
                )
                startActivity(programmingModeIntent)
            } else {
                AppUtil.getInstance().showToast(
                    UILocation.ONLY_LOCAL,
                    context!!.getString(R.string.toastWifiUpBeforeProgrammingMode)
                )
            }
        } else if (id == R.id.action_inspection_mode) {
            val inspectionModeIntent =
                Intent(AppUtil.getDefContext(), RcInspectionActivity::class.java)
            startActivity(inspectionModeIntent)
            return true
        } else if (id == R.id.action_restart_robot) {
            dimmer!!.handleDimTimer()
            AppUtil.getInstance()
                .showToast(UILocation.BOTH, context!!.getString(R.string.toastRestartingRobot))
            requestRobotRestart()
            return true
        } else if (id == R.id.action_configure_robot) {
            val parameters: EditParameters<*> = EditParameters<DeviceConfiguration?>()
            val intentConfigure = Intent(AppUtil.getDefContext(), FtcLoadFileActivity::class.java)
            parameters.putIntent(intentConfigure)
            startActivityForResult(
                intentConfigure,
                LaunchActivityConstantsList.RequestCode.CONFIGURE_ROBOT_CONTROLLER.ordinal
            )
        } else if (id == R.id.action_settings) {
            // historical: this once erroneously used FTC_CONFIGURE_REQUEST_CODE_ROBOT_CONTROLLER
            val settingsIntent =
                Intent(AppUtil.getDefContext(), FtcRobotControllerSettingsActivity::class.java)
            startActivityForResult(
                settingsIntent,
                LaunchActivityConstantsList.RequestCode.SETTINGS_ROBOT_CONTROLLER.ordinal
            )
            return true
        } else if (id == R.id.action_about) {
            val intent = Intent(AppUtil.getDefContext(), FtcAboutActivity::class.java)
            startActivity(intent)
            return true
        } else if (id == R.id.action_exit_app) {

            // Clear backstack and everything to prevent edge case where VM might be
            // restarted (after it was exited) if more than one activity was on the
            // backstack for some reason.
            finishAffinity()

            // For lollipop and up, we can clear ourselves from the recents list too
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {
                val manager = getSystemService(ACTIVITY_SERVICE) as ActivityManager
                val tasks = manager.appTasks
                for (task in tasks) {
                    task.finishAndRemoveTask()
                }
            }

            // Allow the user to use the Control Hub operating system's UI, instead of relaunching the app
            AppAliveNotifier.getInstance().disableAppWatchdogUntilNextAppStart()

            // Finally, nuke the VM from orbit
            AppUtil.getInstance().exitApplication()
            return true
        }
        return super.onOptionsItemSelected(item)
    }

    override fun onConfigurationChanged(newConfig: Configuration) {
        super.onConfigurationChanged(newConfig)
        // don't destroy assets on screen rotation
        updateMonitorLayout(newConfig)
    }

    /**
     * Updates the orientation of monitorContainer (which contains cameraMonitorView and
     * tfodMonitorView) based on the given configuration. Makes the children split the space.
     */
    private fun updateMonitorLayout(configuration: Configuration) {
        val monitorContainer = findViewById<View>(R.id.monitorContainer) as LinearLayout
        if (configuration.orientation == Configuration.ORIENTATION_LANDSCAPE) {
            // When the phone is landscape, lay out the monitor views horizontally.
            monitorContainer.orientation = LinearLayout.HORIZONTAL
            for (i in 0 until monitorContainer.childCount) {
                val view = monitorContainer.getChildAt(i)
                view.layoutParams =
                    LinearLayout.LayoutParams(
                        0,
                        LinearLayout.LayoutParams.MATCH_PARENT,
                        1f /* weight */
                    )
            }
        } else {
            // When the phone is portrait, lay out the monitor views vertically.
            monitorContainer.orientation = LinearLayout.VERTICAL
            for (i in 0 until monitorContainer.childCount) {
                val view = monitorContainer.getChildAt(i)
                view.layoutParams =
                    LinearLayout.LayoutParams(
                        LinearLayout.LayoutParams.MATCH_PARENT,
                        0,
                        1f /* weight */
                    )
            }
        }
        monitorContainer.requestLayout()
    }

    override fun onActivityResult(request: Int, result: Int, intent: Intent) {
        if (request == REQUEST_CONFIG_WIFI_CHANNEL) {
            if (result == RESULT_OK) {
                AppUtil.getInstance().showToast(
                    UILocation.BOTH,
                    context!!.getString(R.string.toastWifiConfigurationComplete)
                )
            }
        }
        // was some historical confusion about launch codes here, so we err safely
        if (request == LaunchActivityConstantsList.RequestCode.CONFIGURE_ROBOT_CONTROLLER.ordinal || request == LaunchActivityConstantsList.RequestCode.SETTINGS_ROBOT_CONTROLLER.ordinal) {
            // We always do a refresh, whether it was a cancel or an OK, for robustness
            shutdownRobot()
            cfgFileMgr!!.activeConfigAndUpdateUI
            updateUIAndRequestRobotSetup()
        }
    }

    fun onServiceBind(service: FtcRobotControllerService) {
        RobotLog.vv(FtcRobotControllerService.TAG, "%s.controllerService=bound", tag)
        controllerService = service
        updateUI!!.setControllerService(controllerService)
        updateUIAndRequestRobotSetup()
        programmingModeManager!!.setState(object : FtcRobotControllerServiceState {
            override fun getWebServer(): WebServer {
                return service.webServer
            }

            override fun getEventLoopManager(): EventLoopManager {
                return service.robot.eventLoopManager
            }
        })
        FtcDashboard.attachWebServer(service.webServer)
    }

    private fun updateUIAndRequestRobotSetup() {
        if (controllerService != null) {
            callback!!.networkConnectionUpdate(controllerService!!.networkConnectionStatus)
            callback!!.updateRobotStatus(controllerService!!.robotStatus)
            // Only show this first-time toast on headless systems: what we have now on non-headless suffices
            requestRobotSetup(
                if (LynxConstants.isRevControlHub()) Runnable {
                    showRestartRobotCompleteToast(
                        R.string.toastRobotSetupComplete
                    )
                } else null
            )
        }
    }

    private fun requestRobotSetup(runOnComplete: Runnable?) {
        if (controllerService == null) return
        var file = cfgFileMgr!!.activeConfigAndUpdateUI
        val hardwareFactory = HardwareFactory(context)
        try {
            hardwareFactory.xmlPullParser = file.xml
        } catch (e: NotFoundException) {
            file = RobotConfigFile.noConfig(cfgFileMgr)
            hardwareFactory.xmlPullParser = file.xml
            cfgFileMgr!!.setActiveConfigAndUpdateUI(false, file)
        }
        val userOpModeRegister = createOpModeRegister()
        eventLoop = FtcEventLoop(hardwareFactory, userOpModeRegister, callback, this)
        val idleLoop = FtcEventLoopIdle(hardwareFactory, userOpModeRegister, callback, this)
        controllerService!!.setCallback(callback)
        controllerService!!.setupRobot(eventLoop, idleLoop, runOnComplete)
        passReceivedUsbAttachmentsToEventLoop()
        AndroidBoard.showErrorIfUnknownControlHub()
        FtcDashboard.attachEventLoop(eventLoop)
    }

    protected fun createOpModeRegister(): OpModeRegister {
        return FtcOpModeRegister()
    }

    private fun shutdownRobot() {
        if (controllerService != null) controllerService!!.shutdownRobot()
    }

    private fun requestRobotRestart() {
        AppUtil.getInstance().showToast(
            UILocation.BOTH,
            AppUtil.getDefContext().getString(R.string.toastRestartingRobot)
        )
        //
        RobotLog.clearGlobalErrorMsg()
        RobotLog.clearGlobalWarningMsg()
        shutdownRobot()
        requestRobotSetup { showRestartRobotCompleteToast(R.string.toastRestartRobotComplete) }
    }

    private fun showRestartRobotCompleteToast(@StringRes resid: Int) {
        AppUtil.getInstance().showToast(UILocation.BOTH, AppUtil.getDefContext().getString(resid))
    }

    private fun checkPreferredChannel() {
        // For P2P network, check to see what preferred channel is.
        if (networkType == NetworkType.WIFIDIRECT) {
            var prefChannel = preferencesHelper!!.readInt(
                getString(com.qualcomm.ftccommon.R.string.pref_wifip2p_channel),
                -1
            )
            if (prefChannel == -1) {
                prefChannel = 0
                RobotLog.vv(
                    tag,
                    "pref_wifip2p_channel: No preferred channel defined. Will use a default value of %d",
                    prefChannel
                )
            } else {
                RobotLog.vv(
                    tag,
                    "pref_wifip2p_channel: Found existing preferred channel (%d).",
                    prefChannel
                )
            }

            // attempt to set the preferred channel.
            RobotLog.vv(tag, "pref_wifip2p_channel: attempting to set preferred channel...")
            wifiDirectChannelChanger = WifiDirectChannelChanger()
            wifiDirectChannelChanger!!.changeToChannel(prefChannel)
        }
    }

    protected fun hittingMenuButtonBrightensScreen() {
        val actionBar = actionBar
        actionBar?.addOnMenuVisibilityListener { isVisible ->
            if (isVisible) {
                dimmer!!.handleDimTimer()
            }
        }
    }

    protected fun initWifiMute(enable: Boolean) {
        if (enable) {
            wifiMuteStateMachine = WifiMuteStateMachine()
            wifiMuteStateMachine!!.initialize()
            wifiMuteStateMachine!!.start()
            motionDetection = MotionDetection(2.0, 10)
            motionDetection!!.startListening()
            motionDetection!!.registerListener { wifiMuteStateMachine!!.consumeEvent(WifiMuteEvent.USER_ACTIVITY) }
        } else {
            wifiMuteStateMachine!!.stop()
            wifiMuteStateMachine = null
            motionDetection!!.stopListening()
            motionDetection!!.purgeListeners()
            motionDetection = null
        }
    }

    override fun onUserInteraction() {
        if (wifiMuteStateMachine != null) {
            wifiMuteStateMachine!!.consumeEvent(WifiMuteEvent.USER_ACTIVITY)
        }
    }

    protected inner class RobotRestarter : Restarter {
        override fun requestRestart() {
            requestRobotRestart()
        }
    }

    protected inner class SharedPreferencesListener : OnSharedPreferenceChangeListener {
        override fun onSharedPreferenceChanged(sharedPreferences: SharedPreferences, key: String) {
            if (key == context!!.getString(R.string.pref_app_theme)) {
                ThemedActivity.restartForAppThemeChange(
                    tag,
                    getString(R.string.appThemeChangeRestartNotifyRC)
                )
            } else if (key == context!!.getString(R.string.pref_wifi_automute)) {
                if (preferencesHelper!!.readBoolean(
                        context!!.getString(R.string.pref_wifi_automute),
                        false
                    )
                ) {
                    initWifiMute(true)
                } else {
                    initWifiMute(false)
                }
            }
        }
    }

    companion object {
        const val tag = "RCActivity"
        private const val REQUEST_CONFIG_WIFI_CHANNEL = 1
        private const val NUM_GAMEPADS = 2
        private var permissionsValidated = false
        @JvmStatic fun setPermissionsValidated() {
            permissionsValidated = true
        }
    }
}
