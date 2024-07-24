---
sidebar_position: 5
---

# 2.5 开机自启动配置

<iframe src="//player.bilibili.com/player.html?aid=700903305&bvid=BV1rm4y1E73q&cid=1196557834&page=15" scrolling="no" border="0" frameborder="no" framespacing="0" width="100%" height="500" allowfullscreen="true"> </iframe>

Ubuntu系统添加自启动程序的方式有多种方法，本章节提供两种方法作为参考。

## 设置自启动Service

1. 创建启动脚本

   使用任何文本编辑器，在`/etc/init.d`目录下创建一个新的启动脚本，假设命名为`your_script_name`，以下是示例脚本的参考内容：

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

2. 设置启动脚本具有可执行权限

   ```
   sudo chmod +x /etc/init.d/your_script_name
   ```

3. 使用update-rc.d命令将脚本添加到系统的启动项中

   ```
   sudo update-rc.d your_script_name defaults
   ```

   

4. 使用systemctl命令启用自启动

   ```
   sudo systemctl enable your_script_name
   ```

5. 重启开发板验证自启动服务程序是否运行正常

    ```
    root@ubuntu:~# systemctl status your_script_name.service 
    ● your_script_name.service - LSB: Start your_service_name at boot time
        Loaded: loaded (/etc/init.d/your_script_name; generated)
        Active: active (exited) since Wed 2023-04-19 15:01:12 CST; 57s ago
        Docs: man:systemd-sysv-generator(8)
        Process: 2768 ExecStart=/etc/init.d/your_script_name start (code=exited, status=0/SUCCESS)
    ```



## 添加到rc.local服务

rc.local是一个系统服务，用于在系统启动时自动执行一些脚本或命令。这个服务在系统启动时会被自动调用，并在系统启动完成后执行一些用户指定的脚本或命令，以便在系统启动时进行自定义配置或操作。

在早期的Linux发行版中，rc.local是系统启动过程中默认运行的最后一个服务。随着systemd的普及，rc.local被视为遗留的系统服务。

通过在`sudo vim /etc/rc.local`文件末尾添加启动命令的方式实现，例如：

```bash
#!/bin/bash -e
#
# rc.local
#re
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
