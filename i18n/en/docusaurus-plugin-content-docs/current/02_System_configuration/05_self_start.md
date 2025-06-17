---
sidebar_position: 5
---
# 2.5 Autostart Programs on Boot

Video: https://www.youtube.com/watch?v=9N_wFttgPeE&list=PLSxjn4YS2IuFUWcLGj2_uuCfLYnNYw6Ld&index=4

There are multiple ways to add autostart programs in Ubuntu system. This section provides two methods as reference.

## Setting up Autostart Service

1. Create a startup script

   Use any text editor and create a new startup script under the `/etc/init.d` directory, assuming it's named `your_script_name`. Here is an example of the script:

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

2. Set the startup script to have execute permission

   ```bash
   sudo chmod +x /etc/init.d/your_script_name
   ```

3. Use the update-rc.d command to add the script to the system's startup items

   ```bash
   sudo update-rc.d your_script_name defaults
   ```

4. Enable autostart using the systemctl command

   ```bash
   sudo systemctl enable your_script_name
   ```

5. Restart the development board to verify if the autostart service program is running correctly

   ```bash
   root@ubuntu:~# systemctl status your_script_name.service 
   ● your_script_name.service - LSB: Start your_service_name at boot time
      Loaded: loaded (/etc/init.d/your_script_name; generated)
      Active: active (exited) since Wed 2023-04-19 15:01:12 CST; 57s ago
      Docs: man:systemd-sysv-generator(8)
      Process: 2768 ExecStart=/etc/init.d/your_script_name start (code=exited, status=0/SUCCESS)
   ```



## Add to rc.local service

rc.local is a system service used to automatically execute scripts or commands when the system starts. This service is automatically called during system startup, and it executes user-specified scripts or commands after the system has finished starting in order to perform custom configurations or operations during system startup.

In earlier Linux distributions, rc.local was the last service to be run during the system startup process by default. With the popularity of systemd, rc.local is considered to be a legacy system service.

This can be achieved by adding the startup command at the end of the `sudo vim /etc/rc.local` file, for example:

```bash
#!/bin/bash -e

# rc.local
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