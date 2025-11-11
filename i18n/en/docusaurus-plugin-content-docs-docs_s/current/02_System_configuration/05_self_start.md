---
sidebar_position: 5
---

# 2.5 Boot Auto-Start Configuration

Video: https://www.bilibili.com/video/BV1rm4y1E73q/?p=15

There are multiple ways to add auto-start programs in Ubuntu. This section provides two methods for reference.

## Configure an Auto-Start Service

1. Create a startup script

   Use any text editor to create a new startup script in the `/etc/init.d` directory. Assume the script is named `your_script_name`. Below is a sample script for reference:

   ```bash
   #!/bin/bash
   
   ### BEGIN INIT INFO
   # Provides:          your_service_name
   # Required-Start:    $all
   # Required-Stop:     
   # Default-Start:     2 3 4 5
   # Default-Stop:      0 1 6
   # Short-Description: Start your_service_name at boot time
   # Description:       Enable service provided by your_service_name
   ### END INIT INFO
   
   /path/to/your/program &
   
   exit 0
   ```

2. Make the startup script executable

   ```
   sudo chmod +x /etc/init.d/your_script_name
   ```

3. Use the `update-rc.d` command to add the script to the system's startup items

   ```
   sudo update-rc.d your_script_name defaults
   ```

4. Enable auto-start using the `systemctl` command

   ```
   sudo systemctl enable your_script_name
   ```

5. Reboot the development board to verify that the auto-start service runs correctly

    ```
    root@ubuntu:~# systemctl status your_script_name.service 
    ● your_script_name.service - LSB: Start your_service_name at boot time
        Loaded: loaded (/etc/init.d/your_script_name; generated)
        Active: active (exited) since Wed 2023-04-19 15:01:12 CST; 57s ago
        Docs: man:systemd-sysv-generator(8)
        Process: 2768 ExecStart=/etc/init.d/your_script_name start (code=exited, status=0/SUCCESS)
    ```



## Add to the rc.local Service

`rc.local` is a system service used to automatically execute scripts or commands during system boot. This service is automatically invoked at system startup and executes user-specified scripts or commands after the system has finished booting, enabling custom configurations or operations at boot time.

In earlier Linux distributions, `rc.local` was the last service executed by default during the system boot process. With the widespread adoption of systemd, `rc.local` is now considered a legacy system service.

You can implement this by adding your startup command at the end of the `/etc/rc.local` file (edited via `sudo vim /etc/rc.local`), for example:

```bash
#!/bin/bash -e
#
# rc.local
#
# This script is executed at the end of each multiuser runlevel.
# Make sure that the script will "exit 0" on success or any other
# value on error.
#
# In order to enable or disable this script just change the execution
# bits.
#
# By default this script does nothing.

# Insert what you need

exit 0
```