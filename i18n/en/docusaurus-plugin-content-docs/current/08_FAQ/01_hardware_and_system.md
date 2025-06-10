---
sidebar_position: 1
---

# 8.1 Hardware, System, and Environment Configuration

For certified accessories and purchase links, please refer to the [Certified Accessories List](https://developer.d-robotics.cc/rdk_doc/Advanced_development/hardware_development/rdk_x3/accessory).

### Q1: What is the D-Robotics RDK Kit?
**A:** D-Robotics Developer Kits, abbreviated as [D-Robotics RDK Kits](https://developer.d-robotics.cc/rdk_doc/), are robot developer kits based on D-Robotics intelligent chips. The main series currently include **RDK X3 (Sunrise X3 Pi)**, **RDK X3 Module**, **RDK X5**, and **RDK Ultra**.

### Q2: How to check the system version of an RDK board?
**A:** After logging into the RDK board, you can use the following commands:
1.  **Check the main system version:**
    ```bash
    cat /etc/version
    ```
    For example, the output might be `2.0.0` or `x3_ubuntu_v1.1.6`.

2.  **Check installed core D-Robotics packages:**
    ```bash
    apt list --installed | grep hobot
    ```
    Or use the `rdkos_info` command (available on newer systems, e.g., 2.1.0 and above):
    ```bash
    rdkos_info
    ```
    **Sample output (RDK OS 2.x, e.g., 2.0.0):**
    ```shell
    root@ubuntu:~# apt list --installed | grep hobot
    hobot-boot/unknown,now 2.0.0-20230530181103 arm64 [installed]
    hobot-bpu-drivers/unknown,now 2.0.0-20230530181103 arm64 [installed]
    # ... other hobot-* packages
    root@ubuntu:~# cat /etc/version
    2.0.0
    ```
    **Sample output (RDK OS 1.x, e.g., 1.1.6):**
    ```shell
    root@ubuntu:~# apt list --installed | grep hobot
    hobot-arm64-boot/unknown,now 1.1.6 arm64 [installed]
    # ... other hobot-arm64-* packages
    root@ubuntu:~# cat /etc/version
    x3_ubuntu_v1.1.6
    ```

### Q3: What is the relationship between different RDK OS versions and hardware platforms?
**A:**
* **RDK OS 2.x and newer (e.g., 2.0.0, 2.1.0, 3.0.x):**
    * Based on D-Robotics open-source Linux packages.
    * Typically supports the corresponding RDK hardware series, e.g., RDK X3 2.x/3.x supports RDK X3 and RDK X3 Module.
* **RDK OS 1.x:**
    * Based on a closed-source Linux system (legacy version).
    * Mainly supports early RDK X3 hardware.

**Important Notes:**
* **Version upgrade:** 1.x systems **cannot** be upgraded to 2.x or newer via `apt`. To upgrade, you must re-flash the new system image ([installation guide](https://developer.d-robotics.cc/rdk_doc/install_os)).
* **TROS compatibility:** Different major TROS versions (e.g., Foxy-based TROS vs. Humble-based TROS) are usually tied to specific RDK OS versions. For example, RDK OS 2.x typically comes with ROS2 Foxy-based TROS, while RDK OS 3.x comes with ROS2 Humble-based TROS.

### Q4: What should I pay attention to when plugging or unplugging the camera?
**A:** **Never plug or unplug the camera while the board is powered on.** Doing so can easily damage the camera module or the mainboard connector. Always disconnect all power before connecting or removing the camera.

### Q5: How to correctly connect the RDK X3 debug serial cable?
**A:** One end of the RDK X3 debug serial cable (usually a white XH connector or corresponding pin header) connects to the DEBUG serial port on the RDK X3 board. The connector usually has a foolproof design (such as a notch or specific pin order), making it hard to connect incorrectly. The other end connects to a USB-to-serial module (e.g., CH340, CP210x), which then connects to your PC via USB.
Connection diagram:
![RDK X3 Serial Connection Diagram](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/08_FAQ/image/hardware_and_system/connect.png)
**Key point:** Ensure the serial module's TX connects to RDK's RX, RX to RDK's TX, and GND to GND.

### Q6: How to connect F37 and GC4663 MIPI cameras to RDK X3? How to verify the connection?
**A:** F37 and GC4663 MIPI camera modules usually connect via a 24-pin FPC (flexible flat cable).
**Connection note:** The FPC cable usually has a blue stiffener on both ends. Make sure the **blue side faces up** (or towards the connector latch, depending on the connector type) when inserting into the board and camera module, and lock the latch.
F37 camera connection diagram:
![F37 Camera to RDK X3 Diagram](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/08_FAQ/image/hardware_and_system/image-X3-PI-Camera.png)

**After connecting:**
1.  Ensure the camera is properly connected and the board is powered on.
2.  **Run the MIPI camera sample (for RDK X3):**
    ```bash
    cd /app/ai_inference/03_mipi_camera_sample # Path may vary by system version
    sudo python3 mipi_camera.py
    ```
    If successful, you should see the camera feed and possible AI results via HDMI or other output.
    Example HDMI output (detecting `teddy bear`, `cup`, and `vase`):
    ![MIPI Camera AI Output Example](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/08_FAQ/image/hardware_and_system/image-20220511181747071.png)

3.  **Check I2C communication with `i2cdetect`:**
    MIPI cameras use I2C for configuration. Use `i2cdetect` to scan devices on the relevant I2C bus (often `i2c-1` or `i2c-2` on RDK X3; check hardware manual or device tree).
    ```bash
    sudo i2cdetect -y -r 1  # Scan i2c-1
    # or sudo i2cdetect -y -r 2 # Scan i2c-2
    ```
    **Expected output:**
    * **F37 (usually at 0x40):**
        ```
              0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
        00:
        ...
        30: -- -- -- -- -- -- -- -- -- -- -- UU -- -- -- --  (UU means kernel driver has claimed it)
        40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --  (0x40 is F37 address)
        ...
        ```
    * **GC4663 (usually at 0x29):**
        ```
              0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
        00:
        ...
        20: -- -- -- -- -- -- -- -- -- 29 -- -- -- -- -- --  (0x29 is GC4663 address)
        30: -- -- -- -- -- -- -- -- -- -- -- UU -- -- -- --
        ...
        ```
    If `i2cdetect` finds the camera's I2C address, it means the camera is recognized at the I2C level.

### Q7: What could cause the board to fail to boot, show nothing on power-up, or keep rebooting? How to troubleshoot?
**A:** These issues are usually related to power supply, boot media (SD card/eMMC), or hardware connections.
* **Insufficient or unstable power:**
    * **Symptoms:** System reboots during U-Boot or early kernel boot without clear error logs; green LED abnormal (e.g., not blinking or always on); HDMI black screen.
        ![U-Boot reboot due to power issue](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/08_FAQ/image/system/image-20230914173433676.png)
        ![Kernel reboot due to power issue](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/08_FAQ/image/system/image-20230914174123619.png)
    * **Troubleshooting:**
        * Use a power adapter that meets board requirements (RDK X3: at least 5V/2A, recommended 5V/3A or higher with QC/PD support).
        * **Do not** power the board from a PC USB port.
        * Use a reliable USB Type-C cable.
        * Refer to the [official accessory list](https://developer.d-robotics.cc/rdk_doc/Advanced_development/hardware_development/rdk_x3/accessory) for recommended adapters.

* **Boot media issues (Micro SD/eMMC):**
    * **Symptoms:** Serial log shows filesystem mount failure, missing partitions, MMC/SD init errors or timeouts.
        ![SD card image format error](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/08_FAQ/image/system/image-20221124194527634.png)
        ![SD card physical/contact error 1](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/08_FAQ/image/system/image-20221124194636213.png)
        ![SD card physical/contact error 2](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/08_FAQ/image/system/image-20221124194721750.png)
    * **Troubleshooting:**
        * Ensure the SD card image is correctly and fully flashed.
        * Try reflashing the system image.
        * Replace with a new, high-quality Micro SD card.
        * Clean the SD card slot and contacts.

* **Accidentally entering U-Boot via serial:**
    * **Symptoms:** System stops at U-Boot prompt (e.g., `hobot>`).
    * **Troubleshooting:** May be caused by unexpected serial input during power-up. Try unplugging the serial cable and rebooting. If at U-Boot prompt, try typing `boot` and pressing Enter.

* **Other hardware or peripheral conflicts:**
    * If above steps fail, remove all unnecessary peripherals (USB devices, expansion boards) and try again.
    * In rare cases, the board itself may be faulty.

* **Detailed troubleshooting:** See the [official forum guide](https://developer.d-robotics.cc/forumDetail/229678192959702636). Connecting a debug serial cable and capturing the full boot log is crucial for diagnosis.

### Q8: What are the power supply requirements for RDK X3?
**A:** RDK X3 is powered via USB Type-C and supports QC (Quick Charge) and PD (Power Delivery) protocols.
* **Recommended:** Use a QC or PD-compatible power adapter.
* **Minimum:** At least **5V DC 2A**. For stable operation, especially with peripherals or high load, use **5V/3A** or higher.
* **Warning:** **Strongly discouraged** to use a PC USB port for power, as it often cannot supply enough current, leading to boot failures, HDMI issues, abnormal LEDs, or reboots.

### Q9: Are there recommended SD card brands or specs for RDK X3?
**A:**
* **Specs:** Use high-speed **C10 (Class 10)** or higher (e.g., UHS Speed Class 1 (U1), U3, Application Performance Class A1/A2) Micro SD cards. Capacity: **16GB or above**.
* **Compatibility:** Old, unbranded, or low-speed SD cards may cause boot failures, slow operation, or unstable data access.
* **Recommended brands (for reference, test and check latest official advice):** Kingston, SanDisk, Samsung, etc.
    * Kingston example: `https://item.jd.com/25263496192.html`
    * SanDisk example: `https://item.jd.com/1875992.html#crumb-wrap`
    (Links are from original docs; verify product and channel before purchase.)

### Q10: What to do if `apt update` fails (e.g., key error, update failure, lock file in use)?
**A:**
#### (1) Source domain or GPG key issues
* **Cause:** The official apt source domain or GPG key may have changed.
* **Error examples:**
    * `Clearsigned file isn't valid, got 'NOSPLIT'`
    * `The repository '...' is no longer signed.`
    * `Could not resolve 'archive.sunrisepi.tech'` (or other old domains)
    * `The following signatures couldn't be verified because the public key is not available: NO_PUBKEY ...`
* **Solution:**
    1.  **Update domain:** Edit `/etc/apt/sources.list.d/sunrise.list` (or other source files), replace old domains (e.g., `archive.sunrisepi.tech`, `sunrise.horizon.cc`) with the latest (`archive.d-robotics.cc`).
        ```bash
        # Replace "old_domain" with the actual old domain in your file
        sudo sed -i 's/old_domain/archive.d-robotics.cc/g' /etc/apt/sources.list.d/sunrise.list
        ```
    2.  **Update GPG key:** Download and install the latest GPG key.
        ```bash
        sudo wget -O /usr/share/keyrings/sunrise.gpg http://archive.d-robotics.cc/keys/sunrise.gpg
        ```
    3.  Try `sudo apt update` again.

#### (2) apt lock file in use
* **Error example:**
    ```
    E: Could not get lock /var/lib/apt/lists/lock. It is held by process XXXX (apt-get)
    N: Be aware that removing the lock file is not a solution and may break your system.
    E: Unable to lock directory /var/lib/apt/lists/
    ```
* **Cause:** The system may be running background updates, or a previous apt operation didn't finish.
* **Solution:**
    1.  **Wait:** Sometimes the background process will finish soon.
    2.  **Kill the process:** If locked for a long time, kill the process holding the lock (the error message shows the PID, e.g., `XXXX`):
        ```bash
        sudo kill XXXX
        ```
    3.  **Remove lock files (with caution):** After confirming no apt/dpkg processes are running, you can remove lock files. **This is risky and may break your package manager; proceed with caution.**
        ```bash
        sudo rm /var/lib/apt/lists/lock
        sudo rm /var/cache/apt/archives/lock
        sudo rm /var/lib/dpkg/lock
        sudo rm /var/lib/dpkg/lock-frontend
        sudo dpkg --configure -a  # Attempt to fix unfinished package configs
        ```
    4.  Try `sudo apt update` again.

### Q11: How to check the status of CPU, BPU, and other hardware units on RDK X3?
**A:** Use the `hrut_somstatus` tool provided by D-Robotics to view real-time system status, including CPU core usage, BPU (AI unit) usage, memory, and chip temperature.
On the board terminal, run:

```bash
sudo hrut_somstatus
```

This command refreshes status at intervals. Press Ctrl+C to exit.

### Q12: How to set up auto-start for RDK applications?
**A:** Two main methods:
1.  **Via `/etc/rc.local` (traditional):**
    Edit this file (create if missing, or enable `rc-local.service`), add your command before `exit 0`, and ensure the script is executable.
    ```bash
    #!/bin/bash -e
    # rc.local
    # Example: Start your application in the background
    # /usr/bin/python3 /home/sunrise/my_app.py &
    exit 0
    ```
    Reference: [RDK Docs - rc.local Autostart](https://developer.d-robotics.cc/rdk_doc/System_configuration/self_start)

2.  **Via `systemd` service (modern, recommended):**
    Create a `.service` file (e.g., `/etc/systemd/system/myapp.service`), define the start command, dependencies, user, restart policy, etc.
    **Example `myapp.service`:**
    ```ini
    [Unit]
    Description=My Application Service
    After=network.target multi-user.target

    [Service]
    User=sunrise
    ExecStart=/usr/bin/python3 /home/sunrise/my_app.py
    Restart=on-failure
    # StandardOutput=append:/var/log/myapp_stdout.log
    # StandardError=append:/var/log/myapp_stderr.log

    [Install]
    WantedBy=multi-user.target
    ```
    Then enable and start the service:
    ```bash
    sudo systemctl daemon-reload
    sudo systemctl enable myapp.service
    sudo systemctl start myapp.service
    # Check status: sudo systemctl status myapp.service
    # View logs (if configured): journalctl -u myapp.service
    ```
    Reference: [RDK Community - systemd Autostart](https://developer.d-robotics.cc/forumDetail/204825652464181760)

**Autostart notes:**
* Ensure script/program paths are correct and executable.
* Handle dependencies (e.g., network, hardware init) and environment variables.
* If a GUI is needed, ensure `DISPLAY` is set and X Server is running.
* Redirect output to log files for troubleshooting.

### Q13: What are the default login accounts and passwords for the board?
**A:** Usually, the following accounts are available:
* **Normal user:** Username `sunrise`, password `sunrise`
* **Superuser (root):** Username `root`, password `root`
  (Note: Actual defaults may vary by image version; check the release notes.)

### Q14: How to mount an NTFS USB drive or HDD on RDK and enable read/write?
**A:** Ubuntu's default NTFS support may be read-only. For full read/write, install `ntfs-3g`:
1.  **Install `ntfs-3g`:**
    ```bash
    sudo apt update
    sudo apt -y install ntfs-3g
    ```
2.  **Mount the NTFS partition:**
    After installation, use `mount` (the system will use `ntfs-3g` automatically).
    * Create a mount point:
        ```bash
        sudo mkdir /mnt/my_ntfs_disk
        ```
    * Mount the device (replace `/dev/sda1` as needed):
        ```bash
        sudo mount /dev/sda1 /mnt/my_ntfs_disk
        # Or explicitly specify type (usually not needed)
        # sudo mount -t ntfs-3g /dev/sda1 /mnt/my_ntfs_disk
        ```
    You should now have read/write access at `/mnt/my_ntfs_disk`.

### Q15: Can RDK boards run VS Code locally? How to use VS Code on PC to connect remotely?
**A:**
* **Local VS Code:** RDK boards (ARM embedded devices) **do not support** running the full VS Code desktop app locally. Official VS Code is mainly for x86_64.
* **Remote development (recommended):**
  Install VS Code on your PC (Windows, macOS, or Linux) and use the **Remote - SSH** extension to connect to the RDK board. This lets you use full VS Code features on your PC, while code runs/compiles on the board.
  **Steps:**
  1.  Install "Remote - SSH" in VS Code on your PC.
  2.  Ensure your PC and RDK board are on the same LAN, and SSH is enabled on the board.
  3.  Configure SSH in VS Code (e.g., `ssh username@board_ip`).
  4.  Once connected, you can open folders on the RDK board directly in VS Code.

### Q16: How to enable and use ADB (Android Debug Bridge) on RDK?
**A:** RDK Ubuntu usually has `adbd` (ADB daemon) built in, but USB mode may need adjustment.
1.  **Check `adbd` service:** Ensure it's running or has a startup script.
2.  **USB mode:** The board's USB Type-C or Micro USB (marked OTG/Device) may need to be set to Device mode (not Host) for ADB. This can sometimes be set via `srpi-config` or by editing device tree/kernel params.
3.  **PC setup:** Install ADB tools (part of Android SDK Platform Tools).
4.  **Connect:** Use a USB cable to connect PC and the board's Device-mode USB port.
5.  **Verify:** On PC, run `adb devices`. If configured correctly, the RDK device should appear.
6.  **Usage:** Once connected, use `adb shell` for terminal, `adb push <local> <remote>` to upload, `adb pull <remote> <local>` to download.

**Note:** Steps may vary by RDK model and system version. Check official docs for details. Sometimes, bootloader update guides may also cover ADB setup.
Reference: [bootloader image update](https://developer.d-robotics.cc/forumDetail/88859074455714818) (may be bootloader-focused; look for ADB-specific docs/posts).

### Q17: What are common ways to transfer files between the board and a PC?
**A:** Several options:
1.  **SCP (Secure Copy, via SSH):**
    * **PC to board:**
        ```bash
        scp /path/to/local_file sunrise@<board_ip>:/path/on/rdk/
        # For folders:
        scp -r /path/to/local_folder sunrise@<board_ip>:/path/on/rdk/
        ```
    * **Board to PC:**
        ```bash
        scp sunrise@<board_ip>:/path/on/rdk/remote_file /path/to/local/
        scp -r sunrise@<board_ip>:/path/on/rdk/remote_folder /path/to/local/
        ```
    SCP is built-in on Linux/macOS; on Windows, use WinSCP, MobaXterm, or Git Bash.

2.  **SFTP (SSH File Transfer):**
    Many FTP clients (FileZilla, WinSCP) support SFTP for GUI file transfer. Use SSH credentials.

3.  **USB storage (U disk/HDD):**
    * Format USB drive to a supported filesystem (FAT32, exFAT, or NTFS with `ntfs-3g`).
    * Copy files on PC, insert into board's USB Host port.
    * On the board, mount the drive (`sudo mount /dev/sda1 /mnt/usb_disk`), then access files.

4.  **ADB (if configured):**
    * **PC to board:**
        ```bash
        adb push C:\local\path\file.txt /remote/path/on/rdk/
        ```
    * **Board to PC:**
        ```bash
        adb pull /remote/path/on/rdk/file.txt C:\local\path\
        ```

5.  **Network sharing (Samba, NFS):**
    Set up Samba or NFS on the board to share directories over LAN. More complex to configure.

6.  **Python HTTP server (for quick sharing):**
    In the desired directory on the board:
    ```bash
    cd /path/to/share
    python3 -m http.server 8000
    ```
    Then access `http://<board_ip>:8000` in your PC browser to download files.

Choose based on file size, frequency, network, and preference. For code/config sync, SCP/SFTP or VS Code Remote-SSH is usually most convenient.

### Q18: What is the default wired IP for RDK X3 in different system versions?
**A:**
* For RDK X3 **RDK OS 2.0.0 and below**: default wired IP is `192.168.1.10/24`
* For RDK X3 **RDK OS 2.1.0 and above**: default wired IP is `192.168.127.10/24`

### Q19: What is the default network IP for RDK X5 in different system versions?
**A:** (Example: RDK OS 3.0.0)
* RDK X5 **3.0.0** wired IP: `192.168.127.10/24`
* RDK X5 **3.0.0** USB Device port (virtual USB NIC): `192.168.128.10/24`

### Q20: What if the desktop goes black during `apt upgrade`?
**A:** It is recommended to perform system updates (`sudo apt update && sudo apt upgrade`) via serial terminal or SSH, not in a graphical terminal. Upgrading desktop-related packages in a GUI terminal may cause display interruption or X Server restart, resulting in a black screen.

### Q21: Why doesn't my PC display anything when connecting RDK X3's HDMI output to the PC's HDMI input/output?
**A:** The RDK X3 HDMI port is **output only**, for sending video to a display (monitor/TV).
PC HDMI ports are usually **output only** as well (for external monitors), unless your PC has a capture card or HDMI input.
* Connecting two outputs (RDK X3 and PC HDMI out) will not work.
* If your PC has HDMI input (e.g., via capture card), ensure it's set to the correct input source and supports the RDK X3's output resolution/format.
For viewing RDK X3 output on a PC, recommended methods:
* **VNC remote desktop:** If using the Desktop version and VNC is enabled, use a VNC client on your PC.
* **Video streaming:** Stream camera or AI results from RDK X3 over the network (RTSP, WebRTC), then view on PC.

### Q22: Why does the board boot with the SD card inserted, but fails to boot after removing it?
**A:** Depends on your RDK model and boot mode:
* **For RDK X3 (standard, SD boot) and X3 Module set to SD boot:** The Micro SD card is the **only storage** for OS and data. The SD card **must remain inserted** for normal operation. Removing it means the system can't find the OS and won't boot.
* **For X3 Module set to eMMC boot:** The board has onboard eMMC storage, and the OS can be flashed to eMMC. In this mode, the SD card is **not required** for boot (unless you want to boot from SD temporarily). If you remove an SD card used as external storage, it won't affect eMMC boot.

Check your board model and current boot media settings.

### Q23: Can the RDK OS Server edition be directly upgraded to the Desktop edition?
**A:** The Server and Desktop editions of RDK OS differ significantly in their pre-installed software packages. Most notably, the Desktop edition includes a graphical user interface (such as the XFCE desktop environment) and related components, while the Server edition omits these to save resources.
* **In theory:** You can manually install all the necessary Desktop packages (such as `xserver-xorg`, `xfce4`, `lightdm`, and their dependencies) on a Server system to "upgrade" it to a graphical environment.
* **Official support and stability:** This manual upgrade path is **not officially provided or recommended**, and systems built this way are not guaranteed or tested for stability and completeness. The manual installation process is complex and prone to missing dependencies or configuration conflicts.
* **Recommended approach:** If you need the full functionality and best experience of the Desktop edition, it is strongly recommended to directly download and flash the official **Desktop system image**. This is the best way to ensure system stability and feature completeness.

### Q24: Why is there no display or abnormal output after connecting an HDMI monitor?
**A:** HDMI display issues can be caused by several factors:
1.  **Monitor compatibility:**
    * Some monitors may not be fully compatible with the specific resolutions or refresh rates output by the RDK board.
    * RDK OS 2.1.0 and above introduced more HDMI resolution options, which may also cause compatibility issues with certain older monitors.
    * Generally, standard 1080p (1920x1080) monitors connected at boot are most likely to work.
2.  **Cable issues:** Ensure the HDMI cable is of good quality and firmly connected. Try replacing the cable.
3.  **RDK system configuration:**
    * For Desktop systems, ensure the graphical interface service (such as LightDM) is running.
    * For Server systems, HDMI may only output the boot logo or console by default, not a graphical desktop.
    * On RDK OS 2.1.0 and above, if you encounter display incompatibility, try connecting via VNC (if enabled) and adjust the HDMI output resolution in the system. See: [HDMI display issues and resolution adjustment](https://developer.d-robotics.cc/forumDetail/204825652464181769)
4.  **Power supply issues:** Severe power instability may prevent the display subsystem from initializing properly.
5.  **Hardware issues:** Rarely, the HDMI port on the board or the monitor itself may be faulty.

### Q25: How to capture EDID information for an unsupported HDMI monitor for technical support?
**A:** If your HDMI monitor does not display correctly, technical support may ask for the monitor's EDID (Extended Display Identification Data) to help diagnose or add compatibility. EDID contains information about the monitor's capabilities, supported resolutions, and timings.
Common ways to obtain EDID:
1.  **Via Linux command-line tools (if the board can boot or be accessed otherwise):**
    * Use tools like `get-edid` and `parse-edid` from the `read-edid` package. Install with `sudo apt install read-edid`.
    * Then try to read the EDID from the connected monitor.
2.  **Using dedicated EDID hardware/software:** Some monitor testing tools or EDID programmers can read and save EDID directly.
3.  **Reading on a PC:** If the monitor works on a Linux or Windows PC, you can try reading its EDID there.
    * On Linux, use `xrandr --props` or `get-edid | parse-edid`.
    * On Windows, use tools like MonitorInfoView (NirSoft) or Phoenix EDID Designer.

Once you have the EDID data (usually a binary file or hex text), provide it to technical support.
For detailed steps on RDK platforms, refer to the official forum guide: [How to provide EDID info for unsupported monitors](https://developer.d-robotics.cc/forumDetail/235046352323895808)

### Q26: What should I do if the SD card is not recognized or is unstable?
**A:** If the SD card is not recognized or is unstable, check the following:
1.  **SD card quality:**
    * Using low-quality, aging, or damaged SD cards is a common cause. Try a new, reliable, high-speed Micro SD card (Class 10, U1/U3).
2.  **SD card slot contact:**
    * Ensure the SD card is fully inserted and making good contact. Remove and clean the contacts with an eraser, and check for debris in the slot.
3.  **SD card compatibility:**
    * While RDK X3 has improved SD card compatibility in newer systems, a few cards may still have issues.
    * For **Sunrise X3 Pi**, if you encounter compatibility issues, try manually flashing the latest **miniboot** image. This also applies to RDK X3.
        See: [SD card compatibility and miniboot update](https://developer.d-robotics.cc/forumDetail/88859074455714818)
4.  **System image or flashing issues:**
    * Ensure the image file is not corrupted and the flashing process is correct. Try re-downloading and using recommended tools (balenaEtcher, Rufus).
5.  **Power supply issues:**
    * Unstable power can indirectly affect SD card recognition and stability.
6.  **Board hardware issues:**
    * Rarely, the SD card controller or slot may be faulty.

If you see logs like "mmc0: error -110 whilst initialising SD card" or "Card did not respond to voltage select", it usually indicates SD card recognition or initialization failure.

### Q27: What should I do if the system stops at the `hobot>` U-Boot command line during boot?
**A:** If the system stops at the `hobot>` prompt, it means the board has entered U-Boot (Universal Boot Loader) command mode instead of booting Linux. Possible causes:
1.  **Serial interference:** If the debug serial port receives unexpected characters or signals (e.g., accidental keypress, terminal software sending control characters) during the first few seconds of boot, it may interrupt U-Boot's auto-boot and drop to the command line.
2.  **Boot order configuration:** U-Boot may be configured to try certain boot media first. If the configuration is changed or the preferred media is missing, it may stop at the command line.
3.  **Boot script issues:** Errors or interruptions in the U-Boot boot script.
4.  **Key press interruption:** Some boards enter U-Boot command mode if a specific key is pressed at boot.

**Solutions:**
* **Simple attempt:** At the `hobot>` prompt, type `boot` and press Enter to try booting Linux.
* **Check serial connection:** Ensure the debug serial connection is stable and free from interference. Try disconnecting the serial cable and rebooting.
* **Check boot media:** Ensure the SD card or eMMC image is intact and bootable.
* **Reset U-Boot environment variables (with caution):** If you suspect environment variables are corrupted, you can try restoring defaults (e.g., `env default -a; saveenv; reset`). **This will erase all custom variables—proceed with caution.**

### Q28: Can Sunrise X3 Pi or RDK X3 be powered via the 40-pin GPIO?
**A:**
* **Sunrise X3 Pi:** According to official documentation, Sunrise X3 Pi **cannot** be powered via the 40-pin GPIO. Its power design may not support this path or protection.
* **RDK X3 (standard/core board):** Some versions of RDK X3 (not Sunrise X3 Pi) **can** be powered via the 5V pin on the 40-pin GPIO. However, this is generally **not recommended** as the main power method due to possible stability and current limitations compared to the dedicated Type-C port.

**General strong recommendation:**
Always **use a compliant power adapter via the board's Type-C (or dedicated power) port**. Powering via non-recommended methods (such as GPIO pins or unsuitable modules) risks insufficient current, unstable voltage, and lack of protection, which may damage hardware or cause instability. Such risks are the user's responsibility.

See related FAQ entries for distinguishing RDK X3 and Sunrise X3 Pi.

### Q29: What are common causes of image flashing failures?
**A:** When flashing a system image to an SD card (using balenaEtcher, Rufus, etc.), failures may be due to:
1.  **Image file issues:**
    * **Not fully extracted:** Ensure you are flashing the raw `.img` file, not a compressed archive.
    * **Incomplete or corrupted download:** Re-download and verify the image's MD5/SHA256 checksum if available.
2.  **SD card issues:**
    * **Damaged or poor-quality card:** Try a new, reliable SD card.
    * **Write protection:** Ensure the SD card's physical write-protect switch is off.
    * **Insufficient capacity:** Ensure the SD card is larger than the extracted image.
3.  **Card reader issues:**
    * Faulty or incompatible reader. Try another reader.
    * Some high-speed SD cards may have issues with old or poor-quality readers.
4.  **Flashing tool or PC environment issues:**
    * **Software version:** Try updating or switching flashing tools.
    * **USB port or drivers:** Try a different USB port; ensure drivers are OK.
    * **OS permissions:** On Windows, run the tool as administrator.
    * **Antivirus/firewall interference:** Temporarily disable security software that may block disk writes.
    * **Windows format prompt:** If Windows prompts to format the SD card during flashing, **always choose "No" or close the dialog**. Formatting will interrupt flashing.
5.  **Cross-verification:**
If possible, try flashing on another PC or with different SD card/reader combinations to isolate the issue.

### Q30: What should I do if I cannot connect to RDK X3's VNC remote desktop?
**A:** If you cannot connect via VNC, check the following:
1.  **System version and VNC service status:**
    * **Desktop image:** Ensure you have flashed the **Desktop** system image. The Server version does not include a GUI or VNC by default.
    * **VNC service enabled:**
        * On RDK OS 2.1.0 and above (or upgraded from 2.0.0), VNC must be **enabled manually** in `srpi-config`. SSH into the board, run `sudo srpi-config`, and enable VNC.
        * On earlier versions, VNC may be enabled by default.
    * **VNC service running:** Check for VNC server processes (e.g., `tightvncserver`, `x11vnc`).
2.  **Network connection:**
    * Ensure your PC and RDK X3 are on the **same LAN**.
    * Ensure you can `ping` the RDK X3's IP from your PC.
    * Check firewalls to ensure VNC ports (default 5900 + display number, e.g., 5901 for :1) are not blocked.
3.  **VNC client configuration:**
    * **IP and port:** Enter the correct IP and port in your VNC client. Port is usually `5900 + N` (N = display number, usually 1, so 5901). Some clients use `192.168.1.10:1`.
    * **Password:** The default VNC password is usually `sunrise` or what you set in `srpi-config`.
4.  **SSH connection check:**
    Before troubleshooting VNC, ensure you can SSH into RDK X3. If SSH fails, resolve network or SSH issues first.
5.  **Resource usage:**
    RDK X3 (especially models without a dedicated GPU) may have high CPU/memory usage running a GUI and VNC. If overloaded, VNC may be slow or fail to connect.

### Q31: Why is there only a black window (no image) when running `/app/pydev_demo/02_camera_hdmi_output` or `03_yolov5_camera_hdmi_output` on RDK X5?
**A:**
1.  **Update packages:** Ensure all `hobot*` packages are up to date. With the correct APT source, run:
    ```bash
    sudo apt update && sudo apt upgrade
    ```
    Then reboot the board.
2.  **Purpose of the demo:** These demos (especially `02_camera_hdmi_output`) are designed to output NV12 camera images directly to HDMI via hardware BT1120, **not** to display images in a window using `cv2.imshow()`. The black/gray window you see may just be a placeholder; the actual image is output directly to the HDMI display.
3.  **Disable desktop environment (for HDMI hardware output):** If running the Desktop version and you want HDMI hardware preview, **you must stop the XFCE desktop environment**:
    ```bash
    sudo systemctl stop lightdm
    ```
    After stopping the desktop, use SSH or serial to run the demos.
4.  **Sensor resolution selection:** Some demo versions prompt you to select sensor resolution (e.g., enter a number 0-4). Follow the on-screen instructions.

### Q32: Why is the HDMI output image upside down or missing YOLOv5 detection boxes when running these demos on RDK X5?
**A:**
* **Upside-down image/no detection boxes (when running under XFCE):** If you run these Python demos in the XFCE desktop and see a window via `cv2.imshow()`, the image may be upside down (depending on OpenCV and sensor orientation), and **YOLOv5 detection boxes may not appear**. This is because `cv2.imshow()` renders on the CPU, while detection boxes may be overlaid via hardware OSD (Overlay Screen Display) directly on the HDMI output.
* **Correct HDMI hardware preview:**
    1.  As in Q31, run `sudo systemctl stop lightdm` to stop the desktop.
    2.  SSH or serial into RDK X5.
    3.  Run the demo. The camera image and hardware OSD overlays (for `03_yolov5_camera_hdmi_output`) will be output directly to the HDMI display.

In short, the expected effect (correct orientation and hardware OSD) requires stopping the desktop and observing via HDMI.

### Q33: What typical kernel log errors indicate SD card contact issues or damage?
**A:** If the Micro SD card has contact issues, is damaged, or incompatible, you may see errors like the following in the serial or `dmesg` logs:
```bash
mmc0: card never left busy state
mmc0: error -110 whilst initialising SD card
mmc_rescan_try_freq: send_status error -110
Card did not respond to voltage select! : -110
mmc0: unrecognised CSD structure version x
mmc0: error -22 whilst initialising SD card
eMMC or SD Card not detected on mmchost 0
MMC Device X not found
no mmc device at slot X
```
Such logs usually mean you should check the SD card, replace it, or check the slot.

### Q34: Does RDK X5 support real-time Linux kernel (RT-Linux)? How to obtain and use it?
**A:** Yes, RDK X5 supports the Preempt-RT kernel, which is important for low-latency, deterministic robotics applications.
* **How to obtain:**
    1.  **System image:** Your RDK X5 system version must support it (e.g., RDK OS 3.0.1 or newer).
        Official images: `https://archive.d-robotics.cc/downloads/os_images/rdk_x5/` (look for Preempt-RT resources under the relevant version).
    2.  **Debian packages:** Precompiled RT kernel `.deb` packages are usually provided.
    3.  **Kernel source:** D-Robotics also provides Preempt-RT kernel source on GitHub.
        See: [D-Robotics x5-kernel-rt on GitHub](https://github.com/D-Robotics/x5-kernel-rt)
* **How to use:**
    1.  Ensure your base RDK X5 system is installed and running.
    2.  Download the RT kernel `.deb` packages (image and headers).
    3.  Transfer them to the board.
    4.  Install with `sudo dpkg -i <kernel_image.deb> <kernel_headers.deb>`.
    5.  Update the bootloader (U-Boot) config as per official docs.
    6.  Reboot.
    7.  Verify with `uname -a` (should show `-rt` in the version).

### Q35: VNC remote desktop on RDK X5 is laggy—how to optimize?
**A:** VNC performance depends on many factors, especially on embedded devices without a dedicated GPU. Try the following:
1.  **Connect an HDMI monitor or dummy plug:**
    * Some systems limit graphics performance if no monitor is detected. Connecting a real HDMI monitor or dummy plug may improve VNC performance.
2.  **Use virtual display software:**
    * Configure a virtual display (e.g., `Xvfb` or with `x11vnc`) so VNC grabs from a virtual screen.
    * See community guides, e.g.: [RDK X5 virtual display configuration (CSDN)](https://blog.csdn.net/weixin_64677511/article/details/142444529)
3.  **Optimize VNC settings:**
    * Lower color depth, compression quality, and disable visual effects to reduce data transfer and rendering load.
    * Ensure a stable, high-bandwidth network (wired is better).
4.  **Lower desktop resolution:** Reducing screen resolution reduces data to transfer and render.
5.  **Close unnecessary graphical apps:** Free up resources for VNC.
6.  **Check system load:** Use `top` or `htop` to check CPU/memory usage. High load will affect VNC performance.

### Q36: What should I do if the board gets too hot under heavy load?
**A:** Overheating can affect stability or damage hardware. Solutions:
1.  **Improve cooling:**
    * **Passive vs. active:** For sustained high load, passive heatsinks may not be enough. Use **active cooling** (heatsink with fan) or install in a well-ventilated case.
    * **Heatsink installation:** Ensure good contact with the chip, and use thermal paste or pads.
2.  **Ensure airflow:** Avoid placing the board in enclosed or poorly ventilated spaces.
3.  **Monitor temperature:**
    * Use commands like `cpu_status_monitor` or read `/sys/class/thermal/thermal_zoneX/temp`.
    * Know the safe operating temperature range for your board.
4.  **Optimize application load:**
    * Reduce unnecessary computation, optimize algorithms, or use lighter models to lower power and heat.
5.  **Check power supply:** Unstable power can cause abnormal operation and indirectly affect temperature.

**Note:** "Low power" does not mean "low temperature"—good cooling is essential for stable operation.

### Q37: How to use D-Robotics RDK-specific Python packages (e.g., `hobot.GPIO`, `hobot_dnn`) in a Conda virtual environment?
**A:** Official `hobot.GPIO`, `hobot_dnn`, etc., are usually precompiled and optimized for the system Python on RDK, and may depend on system libraries and drivers. Using them in Conda or other virtual environments can be challenging due to isolation.

Possible approaches and notes:

1.  **Check for official Conda support or `.whl` files:**
    * Check the latest official docs, community, or GitHub for Conda support or `.whl` files that can be installed via `pip` in Conda. This is ideal.
2.  **Try installing system packages in Conda via `pip` (if no `.whl`):**
    * If the packages are installed in the system Python (e.g., `/usr/lib/python3/dist-packages/`), and your Conda Python version matches, **sometimes** you can try installing via `pip` pointing to the package path, but this is not recommended and rarely works reliably.
3.  **Modify `PYTHONPATH` or `sys.path` (not recommended):**
    * You can add the system package path to `PYTHONPATH` or `sys.path` after activating Conda, but this breaks isolation and can cause conflicts or runtime errors. Not recommended for production.
4.  **Use the system Python environment:**
    * If your project does not require strict isolation, or mainly uses RDK-specific packages, the simplest way is to **use the system Python**.
5.  **Containerization (Docker):**
    * If an official Docker image with these packages is available, use it for reliable isolation and deployment.
6.  **Build from source (if available and permitted):**
    * If source code is available and you have build instructions, you can try building in your Conda environment, but this requires advanced skills.

**Summary:** Prefer official Conda support. If not available, use the system Python. Avoid mixing environments unless you fully understand the risks.

### Q38: What should I do if I get "driver signature error" or "Required key not available" when loading a self-compiled Linux kernel module (`.ko`)?
**A:** Newer Linux kernels, especially with Secure Boot enabled, require kernel modules (`.ko` files) to be signed. If you compile a module without signing, you may see such errors when loading with `insmod` or `modprobe`.

**Typical solution:**

1.  **Generate a signing key pair:**
    Use `openssl` to generate a private and public key:
    ```bash
    openssl req -new -x509 -newkey rsa:2048 -keyout MOK.priv -outform DER -out MOK.der -nodes -days 36500 -subj "/CN=My Module Signing Key/"
    ```
    This creates `MOK.priv` (private) and `MOK.der` (public).

2.  **Sign the kernel module:**
    Use the kernel's `scripts/sign-file` script with your keys:
    ```bash
    sudo /usr/src/linux-headers-$(uname -r)/scripts/sign-file sha256 ./MOK.priv ./MOK.der /path/to/your/module.ko
    ```
    Adjust paths as needed.

3.  **Register the public key with the system's MOK (Machine Owner Key) list:**
    Use `mokutil`:
    ```bash
    sudo mokutil --import MOK.der
    ```
    Set a temporary password when prompted.

4.  **Reboot and enroll the key in MOK Manager:**
    On reboot, enter the blue MOK Manager screen, select "Enroll MOK", and enter the password.

5.  **Load the signed module:**
    After these steps, your module should load without signature errors.

**Important:**
* The above is a general process; details may vary by distro, kernel, and Secure Boot config.
* **Always refer to your distro's and kernel's official documentation on "Kernel Module Signing".**
* D-Robotics RDK docs may also have platform-specific guidance: [RDK Docs - Linux Development - Kernel Headers & Module Compilation](https://developer.d-robotics.cc/documents_rdk/linux_development/kernel_headers) (see the relevant section on module signing).

### Q39: 如何升级RDK X5的MiniBoot？
**A:** 在RDK X5上，可以通过 `srpi-config` 工具来方便地升级MiniBoot（U-Boot的早期引导加载程序部分，负责更底层的硬件初始化和引导）。

**步骤如下：**
1.  **通过SSH或串口登录到RDK X5的系统终端。**
2.  **执行 `srpi-config` 工具：**
    ```bash
    sudo srpi-config
    ```
3.  **导航到MiniBoot更新选项：**
    在 `srpi-config` 的菜单中，通常的路径是：
    * 选择 `1 System Options` (或类似名称的系统选项)
    * 然后选择 `S7 Update MiniBoot` (或类似名称的MiniBoot更新选项，具体编号和名称可能随 `srpi-config` 版本略有调整)
4.  **按照提示进行操作：**
    工具会引导您完成升级过程。这通常需要RDK X5能够连接到互联网，以便下载最新的MiniBoot固件包。
5.  **完成并重启：**
    升级完成后，按照提示退出 `srpi-config` 并重启RDK X5使新的MiniBoot生效。

**验证升级：**
您可以在RDK X5重启后的串口启动日志中，查看U-Boot的版本信息。更新后的MiniBoot通常会显示更新的编译日期和版本号。
例如，更新后的版本信息可能类似 (日期和具体版本号会变化)：
`U-Boot 2022.10+ (Dec 26 2024 - 16:58:41 +0800)`

**注意：** 升级MiniBoot是一个底层固件操作，请确保在稳定的电源和网络环境下进行，并仔细阅读 `srpi-config` 工具的提示信息。

### Q40: 在编译大型项目（如使用gcc/make/cmake/colcon构建ROS2工作空间）或运行内存消耗较大的工具（如`hb_mapper`模型转换）时，遇到内存不足的错误怎么办？
**A:** 内存不足（Out of Memory, OOM）是嵌入式设备或资源受限的开发机上编译大型项目或运行内存密集型应用时常见的问题。以下是一些解决方法：

1.  **增加Swap交换空间：**
    当物理内存(RAM)不足时，系统可以使用硬盘上的一部分空间作为虚拟内存（Swap）。这可以缓解OOM问题，但性能会比物理内存慢很多。
    * **创建并启用Swap文件（示例为创建4GB Swap，大小可根据需求调整）：**
        ```bash
        # 1. 创建一个指定大小的空文件
        sudo fallocate -l 4G /swapfile
        # 2. 设置文件权限
        sudo chmod 600 /swapfile
        # 3. 将该文件设置为Swap区域
        sudo mkswap /swapfile
        # 4. 启用Swap文件
        sudo swapon /swapfile
        # 5. (可选) 验证Swap是否已激活
        swapon --show
        free -h
        ```
    * **使其开机自动挂载：** 编辑 `/etc/fstab` 文件，在末尾添加一行：
        ```
        /swapfile none swap sw 0 0
        ```
    * **关闭Swap（如果需要）：**
        ```bash
        sudo swapoff /swapfile
        sudo rm /swapfile # 如果不再需要，可以删除文件
        # 同时记得从 /etc/fstab 中移除对应行
        ```

2.  **减少编译并行度/线程数：**
    编译过程（尤其是C++项目）通常会启动多个并行的编译任务以加快速度，但这也会消耗大量内存。
    * **`make` 命令：** 使用 `-j` 参数指定并行任务数。例如，单线程编译：
        ```bash
        make -j1
        ```
        可以尝试 `-j2`, `-jN` (N为CPU核心数的一半或更少)。
    * **`colcon build` (ROS2)：**
        * 限制并行包编译数量：
            ```bash
            colcon build --parallel-workers 1
            ```
        * 禁用并行包编译，改为串行编译（更慢但内存占用更低）：
            ```bash
            colcon build --executor sequential
            ```
        * 结合使用：
            ```bash
            colcon build --executor sequential --parallel-workers 1
            ```
    * **`cmake`：** CMake本身不直接控制make的并行度，但最终还是通过make执行。可以在调用make时传递 `-j` 参数。
    * **设置 `MAKEFLAGS` 环境变量（临时）：**
        ```bash
        export MAKEFLAGS="-j1" 
        # 然后执行 colcon build 或其他编译命令
        ```

3.  **针对 `hb_mapper` (地平线模型转换工具)：**
    * 在模型转换的 `yaml` 配置文件中，查找是否有类似 `compiler_parameters` -> `jobs: 1` 的选项，用以限制模型编译（例如ONNX到BIN模型过程中某些阶段）的并行进程数。具体参数名请查阅最新的算法工具链文档。

4.  **关闭不必要的后台服务和应用程序：**
    在进行编译或运行内存密集型任务前，关闭其他占用内存的程序（如图形界面、浏览器、其他服务等），以释放更多物理内存。

5.  **使用更高配置的开发机/服务器进行编译：**
    如果是在x86开发机上为RDK进行交叉编译，而开发机本身内存也有限，可以考虑使用内存配置更高的机器。对于板端编译，如果资源实在不足，交叉编译是更好的选择。

6.  **分步编译/模块化编译：**
    对于非常大的项目，如果构建系统支持，可以尝试只编译项目的一部分，或者将项目分解为更小的模块独立编译。

选择哪种方法或组合取决于具体的错误信息、可用资源以及对编译时间的要求。增加Swap通常是比较通用的缓解方法。

### Q41: RDK相关问题进行预排查的通用建议有哪些？
**A:** 在遇到RDK相关问题并寻求帮助前，建议进行以下预排查：
1.  **查阅最新官方手册：** 确保您参考的是官方最新版本的用户手册、开发文档和发行说明。官方文档通常会包含最新的信息和已知问题的解决方案。您可以从地平线开发者社区获取最新文档：[https://developer.d-robotics.cc/information](https://developer.d-robotics.cc/information)
2.  **更新系统及相关软件包：** 许多问题可能在较新的软件版本中得到修复。请确保您的RDK板卡上的操作系统以及所有`hobot-*`、`tros-*`等关键软件包都已更新到最新稳定版本。通常可以通过以下命令进行更新：
    ```bash
    sudo apt update && sudo apt upgrade
    ```
    在提问时，请一并提供通过 `rdkos_info`、`apt list --installed | grep hobot` 等命令获取的当前系统和软件包版本信息。
3.  **仔细检查硬件连接：** 确保所有硬件连接都牢固可靠，包括电源线、SD卡、调试串口线、摄像头排线、网络线以及其他外设连接。接触不良是许多问题的根源。
4.  **提供完整的问题复现信息：** 当您向社区或技术支持提问时，请尽可能提供以下信息：
    * **清晰的问题描述：** 遇到了什么问题？期望的结果是什么？实际观察到的现象是什么？
    * **RDK硬件型号和系统版本：** 例如RDK X5, RDK OS 3.0.1。
    * **相关的软件包版本。**
    * **详细的复现步骤：** 一步一步说明如何操作才能触发问题。
    * **完整的错误日志或截图：** 包括串口打印、dmesg信息、应用程序的报错输出等。
    * **您已尝试过的解决方法及其结果。**
    提供充分的信息有助于他人更快地理解和定位您的问题。

### Q42: Docker镜像、OE包或嵌入式开发Samples包下载失败或速度慢怎么办？
**A:**
1.  **Docker镜像（例如用于算法工具链、交叉编译环境）：**
    * **官方来源：** Docker镜像通常首发于Docker Hub。地平线官方也可能在自己的服务器或特定的开发者社区资源帖中提供部分关键镜像的下载链接或拉取方式。
    * **网络问题：** 如果从Docker Hub拉取速度慢或失败，可能是由于网络限制或国际带宽问题。可以尝试配置Docker使用国内的镜像加速器服务（如阿里云、DaoCloud、网易蜂巢等都提供此类服务）。
    * **社区资源：** 关注地平线开发者社区的公告或资源下载区，有时会提供针对国内用户的镜像获取方案。例如，此帖曾提供过相关资源：[地平线开发者社区论坛相关帖子](https://developer.d-robotics.cc/forumDetail/136488103547258769) (请确认链接及内容的最新有效性)。
2.  **OE (OpenEmbedded) 包 / BSP (Board Support Package)：**
    * OE编译环境相关的包或完整的BSP（包含内核源码、驱动、文件系统构建脚本等）通常体积较大。如果官方提供直接下载，请确保您的网络连接稳定且具有足够的带宽。
    * 这些资源一般会在开发者社区的“资源中心”板块或对应RDK型号的产品文档页提供下载链接。
3.  **嵌入式开发Samples包（示例代码）：**
    * 示例代码包可能作为BSP的一部分提供（例如在BSP解压后的 `bsp/samples/` 或类似目录下）。
    * 也可能作为独立的SDK、代码仓库（如GitHub上的 `D-Robotics` 组织）或压缩包提供。
    * 请仔细查阅对应RDK型号和版本的官方文档或快速入门指南，以找到获取官方示例代码的正确途径。
4.  **通用下载建议：**
    * **使用下载工具：** 对于较大的文件，建议使用支持断点续传的下载工具。
    * **检查网络环境：** 如果您在公司或机构网络下，确认是否有防火墙、代理服务器或网络策略限制了大文件的下载或访问特定域名。
    * **错峰下载：** 尝试在网络负载较低的时段进行下载。
    * **官方渠道优先：** 始终优先从地平线官方开发者社区、官方文档中提供的链接或官方GitHub仓库获取各类开发资源，以确保文件的正确性、完整性和安全性。

### Q43: 为RDK进行交叉编译的环境应该如何配置？
**A:** 为RDK板卡（通常是ARM架构）上的应用程序进行交叉编译，一般需要在x86架构的Linux开发主机（推荐使用Ubuntu LTS版本，如Ubuntu 20.04或22.04）上配置交叉编译工具链和相应的目标系统SDK（Sysroot）。具体配置步骤会因您要编译的程序类型（例如，普通的Linux C/C++程序、ROS/TROS功能包）以及目标RDK的型号和系统版本而有所不同。

1.  **编译普通Linux C/C++应用程序：**
    * **获取交叉编译工具链：** 地平线官方会为每个RDK系列（如X3、X5、Ultra）提供相应的交叉编译工具链（例如，包含`aarch64-linux-gnu-gcc`, `aarch64-linux-gnu-g++`等工具）。这个工具链可能作为SDK的一部分提供，或者需要从开发者社区单独下载。
    * **安装与配置工具链：** 按照官方文档的指引，将下载的工具链压缩包解压到您开发主机上的一个合适路径（例如 `/opt/toolchains/`）。然后，需要将工具链的 `bin` 目录（包含编译器等可执行文件）添加到您开发主机的 `PATH` 环境变量中，这样系统才能找到这些交叉编译命令。
    * **准备Sysroot：** 交叉编译不仅需要编译器，还需要目标板卡系统环境中的库文件（如glibc, libstdc++, 以及其他依赖库）和头文件。这部分内容集合称为Sysroot。Sysroot可以从官方提供的RDK SDK中提取，或者从一个已经烧录好系统的RDK板卡的根文件系统中复制得到。在编译时，需要通过编译器的 `--sysroot=<path_to_sysroot>` 参数来指定Sysroot的路径。
    * **使用CMake进行交叉编译：** 如果您的项目使用CMake作为构建系统，推荐创建一个CMake工具链配置文件（toolchain file，例如 `aarch64-rdk.cmake`）。在这个文件中，您需要指定：
        * 目标系统名称 (`CMAKE_SYSTEM_NAME` 通常设为 `Linux`)。
        * 目标处理器架构 (`CMAKE_SYSTEM_PROCESSOR` 通常设为 `aarch64`)。
        * C交叉编译器 (`CMAKE_C_COMPILER`) 和 C++交叉编译器 (`CMAKE_CXX_COMPILER`) 的完整路径。
        * Sysroot路径 (`CMAKE_SYSROOT`)。
        * 查找库和头文件的相关路径设置 (`CMAKE_FIND_ROOT_PATH`)。
        然后在运行CMake配置项目时，通过 `-DCMAKE_TOOLCHAIN_FILE=/path/to/your/aarch64-rdk.cmake` 参数来指定使用这个工具链文件。
    * **参考官方手册：** 详细的交叉编译环境搭建步骤、工具链文件示例以及编译参数，请务必参考您所使用的RDK型号和版本的官方《用户手册》或《SDK开发指南》中关于“Linux应用开发”或“交叉编译环境搭建”的章节。

2.  **编译ROS/TROS功能包：**
    * **使用官方提供的Docker交叉编译环境（强烈推荐）：** 这是为TROS功能包进行交叉编译**最推荐且最便捷**的方式。地平线官方通常会提供预配置好的Docker镜像，这些镜像中已经集成了：
        * 特定TROS版本（如Foxy, Humble）所需的交叉编译工具链。
        * Ament/Colcon等ROS构建工具。
        * 目标板卡TROS环境对应的所有基础ROS库和依赖项的交叉编译版本。
        * **操作流程：**
            1.  从官方渠道（如Docker Hub或地平线官方服务器）拉取对应TROS版本的交叉编译Docker镜像。
            2.  按照官方文档的指引启动Docker容器，并将您的ROS工作区源代码目录挂载到容器内部。
            3.  在Docker容器的终端内，使用 `colcon build` 配合适当的交叉编译参数（通常Docker环境已预设好）来编译您的工作区。
        * **参考官方手册：** TROS用户手册中关于“源码安装”、“开发者指南”或“交叉编译”的章节通常会有详细的Docker使用方法和命令示例。例如，此链接可能包含相关信息：[TROS手册 - 交叉编译Docker参考](https://developer.d-robotics.cc/rdk_doc/Robot_development/quick_start/cross_compile) (请确认链接的有效性和相关性)。
    * **手动配置ROS/TROS交叉编译环境 (极不推荐，非常复杂且极易出错)：** 如果不使用官方提供的Docker环境，手动从零开始搭建一个完整的ROS/TROS交叉编译环境是一项非常复杂和耗时的工作。您需要自行交叉编译ROS的所有核心组件、消息类型、依赖库，并为Colcon等构建工具配置大量的交叉编译参数和环境变量。这通常只适用于有深厚交叉编译和ROS构建系统经验的开发者。

**通用交叉编译建议：**
* **仔细阅读官方文档：** 针对您使用的RDK型号和目标系统版本，务必以官方最新发布的开发文档、SDK说明和移植指南为准。
* **保持环境一致性：** 交叉编译环境中所使用的库（尤其是系统库和核心依赖库）的版本，应尽可能与目标RDK板卡上实际运行的库版本保持一致或兼容，以避免运行时出现链接错误或行为不一致的问题。
* **Sysroot的正确配置至关重要：** 无论是编译普通Linux程序还是ROS包，正确配置和使用Sysroot是交叉编译成功的关键环节。