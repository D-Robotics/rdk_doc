---
sidebar_position: 4
---

# 8.4 多媒体处理与应用

本节主要解答与地瓜RDK板卡上视频编解码、音频处理以及其他多媒体功能相关的常见疑问。

## 视频编解码

### Q1: 开发板解码RTSP视频流时报错（如下图所示），可能是什么原因？
![RTSP解码报错图片](../../static/img/08_FAQ/image/multimedia/image-20220728110439753.png)
**A:** RTSP视频流解码报错，常见原因及解决方法如下：
1.  **码流缺少PPS和SPS参数信息：**
    * **原因：** 推流服务器推送的RTSP码流（尤其是H.264格式）中必须包含`PPS` (Picture Parameter Set) 和 `SPS` (Sequence Parameter Set) 参数信息，解码器需要这些信息来正确解析视频。
    * **解决方法：**
        * 如果您使用`ffmpeg`从视频文件（如 `.mp4`, `.avi`）推流，建议在命令中添加 `-bsf:v h264_mp4toannexb` (H.264 Bitstream Filter: MP4 to Annex B) 选项（注意：较新版本的ffmpeg中 `-vbsf` 已被 `-bsf:v` 替代）。这个过滤器会自动为码流添加`PPS`和`SPS`信息。
            **`ffmpeg`推流命令示例：**
            ```
            ffmpeg -re -stream_loop -1 -i xxx.mp4 -c:v copy -bsf:v h264_mp4toannexb -f rtsp rtsp://192.168.1.195:8554/h264_stream
            ```
            (请将 `xxx.mp4` 替换为您的视频文件名，并将RTSP服务器地址 `rtsp://192.168.1.195:8554/h264_stream` 替换为实际地址。)
2.  **分辨率支持限制：**
    * 目前RDK板卡对RTSP视频流的解码可能仅支持到特定的分辨率，例如 **1080p (1920x1080)**。请确认您的RTSP流分辨率是否在此支持范围内。查阅您板卡型号的具体文档以获取准确支持列表。
3.  **推流软件兼容性：**
    * **不推荐使用VLC直接推流：** 使用VLC软件直接进行RTSP推流可能无法成功被RDK解码，原因是VLC在某些配置下可能不支持在推流时主动添加或确保`PPS`和`SPS`信息。建议使用`ffmpeg`或其他能确保码流参数完整性的专业推流工具。

## Audio 常见问题

### Q2: RDK板卡上如何区分和使用USB声卡与板载声卡？特别是当同时连接了多种音频设备时。
**A:** 当RDK板卡上同时连接了板载声卡（例如通过音频子板）和USB声卡时，Linux音频系统（ALSA）会为它们分配不同的声卡序号。您需要知道正确的声卡序号才能精确控制特定的音频设备。

1.  **查看已识别的声卡及其序号：**
    使用以下命令可以列出系统中所有已识别的声卡及其对应的序号和名称：
    ```bash
    cat /proc/asound/cards
    ```
    *
    **示例输出 (假设USB声卡先注册，板载声卡后注册)：**
    ```text
     0 [RC08          ]: USB-Audio - ROCWARE RC08
                          ROCWARE RC08 at usb-xhci-hcd.2.auto-1.2, high speed
     1 [duplexaudio   ]: simple-card - duplex-audio
                          duplex-audio
    ```
    在这个示例中：
    * USB声卡 `ROCWARE RC08` 被分配为声卡序号 **0**。
    * 板载声卡 `duplexaudio` (这通常是RDK音频子板的名称) 被分配为声卡序号 **1**。
    * **注意：** 声卡序号的分配顺序可能因设备插入顺序、驱动加载顺序等因素而变化。如果USB声卡是在系统启动后插入的，它可能会获得一个较大的序号。

2.  **使用 `amixer` 或 `tinymix` 指定声卡进行操作：**
    * 当您使用 `amixer` (ALSA Mixer command-line utility) 或 `tinymix` 等工具来查看或调整音频参数时，如果不指定声卡（card）和设备（device）编号，它们通常会默认操作序号为0的声卡。
    * 要操作特定的声卡，需要使用 `-c <card_number>` (或 `-c<card_number>`) 参数指定声卡序号，以及可能需要的 `-D hw:<card_number>` 或 `-d <device_number>` 参数。
    * **查看特定声卡（如上述示例中的板载声卡，序号为1）的控件 (controls)：**
        ```bash
        amixer -c 1 controls 
        # 或者使用硬件设备名: amixer -D hw:1 controls
        ```
        *
    * **获取或设置特定声卡上控件的值 (例如，获取板载声卡序号1上名为 'ADC PGA Gain' 的第一个控件的值)：**
        ```bash
        amixer -c 1 sget 'ADC PGA Gain',0
        ```
        *
        要设置值，可以使用 `sset` 代替 `sget`，例如：`amixer -c 1 sset 'ADC PGA Gain',0 80%`。

通过以上方法，您可以准确地识别并控制连接到RDK板卡上的不同音频设备。

### Q3: RDK X3系列的音频子板如何与USB声卡共存并同时使用（例如，让PulseAudio识别和管理它们）？
**A:** 如果您希望在RDK X3上同时使用板载的音频子板（例如基于WM8960芯片的）和外接的USB声卡，并且让PulseAudio等上层音频服务能够识别和管理它们，您可能需要进行一些配置。

以下步骤以WM8960音频子板和USB全双工声卡为例进行说明：

1.  **确保音频子板正常工作：**
    * 首先，根据对应音频子板的教程，确保其驱动已正确加载，并且在单独使用时可以正常录音和播放。

2.  **接入USB声卡并识别新增节点：**
    * 将USB声卡连接到RDK X3的USB接口。等待系统加载驱动。
    * 观察 `/dev/snd/` 目录下新增的PCM设备节点。ALSA为每个声卡的每个PCM设备（播放、录音）创建节点。
        ```bash
        ls /dev/snd/
        ```
        *
        **示例输出 (假设 `controlC0`, `pcmC0D0c`, `pcmC0D0p`, `pcmC0D1c`, `pcmC0D1p` 是音频子板的节点，而 `pcmC1D0c`, `pcmC1D0p` 是新接入的USB声卡的节点)：**
        ```text
        by-path  controlC0  pcmC0D0c  pcmC0D0p  pcmC0D1c  pcmC0D1p  pcmC1D0c  pcmC1D0p  timer
        ```
        在这个例子中：
        * `pcmC0...` 通常对应声卡0 (card 0)。`D0c` 表示设备0的录音(capture)端点，`D0p` 表示设备0的播放(playback)端点。`D1c`, `D1p` 可能表示声卡0上的第二个PCM设备（例如HDMI音频输出）。
        * `pcmC1D0c`, `pcmC1D0p` 则对应声卡1 (card 1)，即新接入的USB声卡。如果USB声卡是全双工且只有一个PCM设备，它通常会表现为一个录音端点和一个播放端点。

3.  **修改PulseAudio配置文件 (`/etc/pulse/default.pa`)：**
    * 为了让PulseAudio能够同时加载和使用这两个声卡，您需要编辑其默认配置文件。
    * 找到文件中加载ALSA声卡源（source, 用于录音）和槽（sink, 用于播放）的模块部分，通常在 `.ifexists module-udev-detect.so` 块内或 `.else` 块内。
    * 在已有的 `load-module module-alsa-source` 和 `load-module module-alsa-sink` 行之后，为您的USB声卡（假设它是声卡1，设备0，根据 `cat /proc/asound/cards` 确认）添加新的加载指令。

    **修改 `/etc/pulse/default.pa` 示例：**
    ```apacheconf
    # ... (文件其他内容) ...

    .ifexists module-udev-detect.so
    # load-module module-udev-detect tsched=0 # 或者类似这行

    ### Existing ALSA Sink/Source for onboard audio (card 0)
    ### 请根据您板载声卡的实际配置调整 device=hw:X,Y 中的 X 和 Y
    ### 例如，如果板载播放是 card 0, device 1; 板载录音是 card 0, device 0
    load-module module-alsa-sink device=hw:0,1 mmap=false tsched=0 fragments=2 fragment_size=960 rate=48000 channels=2 rewind_safeguard=960
    load-module module-alsa-source device=hw:0,0 mmap=false tsched=0 fragments=2 fragment_size=960 rate=48000 channels=2

    ### Add these lines for the USB sound card (assuming it's card 1, device 0 for both playback and capture)
    ### 注意：这里的 device=hw:1,0 是基于前面 ls /dev/snd/ 和 cat /proc/asound/cards 的推断，
    ### 实际应根据 `cat /proc/asound/cards` 确认USB声卡的card number (X)，
    ### 并根据 `aplay -l` 和 `arecord -l` 确认其播放和录音的device number (Y)。
    load-module module-alsa-sink device=hw:1,0 # 用于USB声卡播放
    load-module module-alsa-source device=hw:1,0 # 用于USB声卡录音

    .else
    # ... (Fallback configuration if udev-detect is not available) ...
    # You might need to add similar lines here if this block is active
    ### Fallback sink
    load-module module-alsa-sink # Default sink
    ### Fallback source
    load-module module-alsa-source device=hw:0,0 # Example for onboard capture

    ### Add for USB sound card if udev is not used
    # load-module module-alsa-sink device=hw:1,0
    # load-module module-alsa-source device=hw:1,0
    .endif

    # ... (文件其他内容) ...
    ```
    *
    **重要说明：**
    * `device=hw:X,Y` 中的 `X` 是声卡序号 (Card Number)，`Y` 是PCM设备序号 (Device Number)。您需要根据 `cat /proc/asound/cards` (查看声卡X) 和 `aplay -l` / `arecord -l` (查看设备Y) 的输出来确定USB声卡实际的 `X` 和 `Y` 值。
    * 上述示例中的 `mmap=false tsched=0 fragments=2 fragment_size=960 rate=48000 channels=2 rewind_safeguard=960` 等参数是针对特定音频子板优化的，对于USB声卡，您可能不需要这么多参数，或者可以先尝试只用 `device=hw:X,Y`。如果遇到声音卡顿或爆音，再尝试调整这些参数。

4.  **保存配置并重启：**
    * 保存对 `/etc/pulse/default.pa` 文件的修改。
    * 重启RDK开发板使PulseAudio重新加载配置。
    * 或者，尝试重启PulseAudio服务（如果知道如何操作且系统支持，例如 `systemctl --user restart pulseaudio.service` 或 `pulseaudio -k && pulseaudio --start`，但这可能不如重启板卡干净）。

重启后，您应该可以在系统的声音设置（如果使用桌面环境）或通过 `pactl list sources` / `pactl list sinks` 命令看到两个声卡的输入和输出设备，并能选择使用它们。
