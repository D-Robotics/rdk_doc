---
sidebar_position: 4
---

# 8.4 Multimedia Processing and Applications

This section addresses frequently asked questions related to video codec, audio processing, and other multimedia features on the Digua RDK board.

## Video Codec

### Q1: What could cause errors when decoding RTSP video streams on the development board (as shown in the image below)?
![RTSP decoding error image](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/08_FAQ/image/multimedia/image-20220728110439753.png)
**A:** Common causes and solutions for RTSP video stream decoding errors include:

1.  **Missing PPS and SPS parameter information in the stream:**
    * **Reason:** The RTSP stream (especially H.264 format) pushed by the streaming server must contain `PPS` (Picture Parameter Set) and `SPS` (Sequence Parameter Set) information. The decoder needs these to correctly parse the video.
    * **Solution:**
        * If you use `ffmpeg` to stream from a video file (such as `.mp4`, `.avi`), add the `-bsf:v h264_mp4toannexb` (H.264 Bitstream Filter: MP4 to Annex B) option to your command (note: in newer ffmpeg versions, `-vbsf` is replaced by `-bsf:v`). This filter will automatically add `PPS` and `SPS` information to the stream.
            **Example `ffmpeg` streaming command:**
            ```
            ffmpeg -re -stream_loop -1 -i xxx.mp4 -c:v copy -bsf:v h264_mp4toannexb -f rtsp rtsp://192.168.1.195:8554/h264_stream
            ```
            (Replace `xxx.mp4` with your video file name and `rtsp://192.168.1.195:8554/h264_stream` with your actual RTSP server address.)
2.  **Resolution support limitations:**
    * The RDK board may only support decoding RTSP video streams up to certain resolutions, such as **1080p (1920x1080)**. Please confirm that your RTSP stream resolution is within the supported range. Refer to your board's documentation for the exact supported list.
3.  **Streaming software compatibility:**
    * **VLC is not recommended for direct streaming:** Using VLC to stream RTSP may not work with RDK decoding, as VLC may not always add or ensure `PPS` and `SPS` information in the stream. It is recommended to use `ffmpeg` or other professional streaming tools that ensure complete stream parameters.

## Common Audio Questions

### Q2: How to distinguish and use USB sound cards and onboard sound cards on the RDK board, especially when multiple audio devices are connected?
**A:** When both onboard sound cards (e.g., via an audio sub-board) and USB sound cards are connected to the RDK board, the Linux audio system (ALSA) assigns different card numbers to each. You need to know the correct card number to control a specific audio device.

1.  **View recognized sound cards and their numbers:**
    Use the following command to list all recognized sound cards and their corresponding numbers and names:
    ```bash
    cat /proc/asound/cards
    ```
    **Example output (assuming USB sound card is registered first, onboard sound card second):**
    ```text
     0 [RC08          ]: USB-Audio - ROCWARE RC08
                          ROCWARE RC08 at usb-xhci-hcd.2.auto-1.2, high speed
     1 [duplexaudio   ]: simple-card - duplex-audio
                          duplex-audio
    ```
    In this example:
    * USB sound card `ROCWARE RC08` is assigned card number **0**.
    * Onboard sound card `duplexaudio` (usually the RDK audio sub-board) is assigned card number **1**.
    * **Note:** The assignment order may change depending on device insertion order, driver loading order, etc. If the USB sound card is plugged in after system boot, it may get a higher card number.

2.  **Use `amixer` or `tinymix` to specify the card for operations:**
    * When using `amixer` (ALSA Mixer command-line utility) or `tinymix` to view or adjust audio parameters, if you do not specify the card and device number, they usually operate on card number 0 by default.
    * To operate a specific sound card, use the `-c <card_number>` (or `-c<card_number>`) parameter to specify the card number, and possibly `-D hw:<card_number>` or `-d <device_number>`.
    * **View controls for a specific sound card (e.g., onboard sound card, card number 1):**
        ```bash
        amixer -c 1 controls 
        # Or use hardware device name: amixer -D hw:1 controls
        ```
    * **Get or set the value of a specific control on a sound card (e.g., get the value of 'ADC PGA Gain' on onboard sound card number 1):**
        ```bash
        amixer -c 1 sget 'ADC PGA Gain',0
        ```
        To set a value, use `sset` instead of `sget`, for example: `amixer -c 1 sset 'ADC PGA Gain',0 80%`.

With these methods, you can accurately identify and control different audio devices connected to the RDK board.

### Q3: How can the RDK X3 series audio sub-board coexist and be used simultaneously with a USB sound card (e.g., managed by PulseAudio)?
**A:** If you want to use both the onboard audio sub-board (e.g., WM8960 chip-based) and an external USB sound card on the RDK X3, and have PulseAudio recognize and manage them, some configuration is required.

The following steps use the WM8960 audio sub-board and a USB full-duplex sound card as an example:

1.  **Ensure the audio sub-board works properly:**
    * First, follow the sub-board's guide to ensure its driver is loaded and it works for recording and playback when used alone.

2.  **Connect the USB sound card and identify new nodes:**
    * Connect the USB sound card to the RDK X3's USB port. Wait for the system to load the driver.
    * Check the `/dev/snd/` directory for new PCM device nodes. ALSA creates nodes for each card's PCM device (playback, capture).
        ```bash
        ls /dev/snd/
        ```
        **Example output (assuming `controlC0`, `pcmC0D0c`, `pcmC0D0p`, `pcmC0D1c`, `pcmC0D1p` are for the audio sub-board, and `pcmC1D0c`, `pcmC1D0p` are for the USB sound card):**
        ```text
        by-path  controlC0  pcmC0D0c  pcmC0D0p  pcmC0D1c  pcmC0D1p  pcmC1D0c  pcmC1D0p  timer
        ```
        In this example:
        * `pcmC0...` usually corresponds to card 0. `D0c` means device 0 capture, `D0p` means device 0 playback. `D1c`, `D1p` may be a second PCM device (e.g., HDMI audio output).
        * `pcmC1D0c`, `pcmC1D0p` correspond to card 1, i.e., the new USB sound card. If the USB sound card is full-duplex with only one PCM device, it will have one capture and one playback endpoint.

3.  **Modify PulseAudio configuration file (`/etc/pulse/default.pa`):**
    * To let PulseAudio load and use both sound cards, edit its default configuration file.
    * Find the module loading section for ALSA sources (for capture) and sinks (for playback), usually inside the `.ifexists module-udev-detect.so` or `.else` block.
    * After the existing `load-module module-alsa-source` and `load-module module-alsa-sink` lines, add new lines for your USB sound card (assuming it is card 1, device 0; confirm with `cat /proc/asound/cards`).

    **Example `/etc/pulse/default.pa` modification:**
    ```apacheconf
    # ... (other content) ...

    .ifexists module-udev-detect.so
    # load-module module-udev-detect tsched=0 # or similar

    ### Existing ALSA Sink/Source for onboard audio (card 0)
    ### Adjust device=hw:X,Y according to your actual configuration
    ### For example, if onboard playback is card 0, device 1; onboard capture is card 0, device 0
    load-module module-alsa-sink device=hw:0,1 mmap=false tsched=0 fragments=2 fragment_size=960 rate=48000 channels=2 rewind_safeguard=960
    load-module module-alsa-source device=hw:0,0 mmap=false tsched=0 fragments=2 fragment_size=960 rate=48000 channels=2

    ### Add these lines for the USB sound card (assuming it's card 1, device 0 for both playback and capture)
    ### Note: device=hw:1,0 is based on previous ls /dev/snd/ and cat /proc/asound/cards,
    ### confirm the actual card number (X) and device number (Y) with `cat /proc/asound/cards`, `aplay -l`, and `arecord -l`.
    load-module module-alsa-sink device=hw:1,0 # For USB sound card playback
    load-module module-alsa-source device=hw:1,0 # For USB sound card capture

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

    # ... (other content) ...
    ```
    **Important notes:**
    * In `device=hw:X,Y`, `X` is the card number, `Y` is the PCM device number. Use `cat /proc/asound/cards` (for X) and `aplay -l` / `arecord -l` (for Y) to confirm the actual values for your USB sound card.
    * The parameters like `mmap=false tsched=0 fragments=2 fragment_size=960 rate=48000 channels=2 rewind_safeguard=960` are optimized for specific audio sub-boards. For USB sound cards, you may not need all these parameters; try with just `device=hw:X,Y` first. If you encounter audio glitches, try adjusting these parameters.

4.  **Save configuration and restart:**
    * Save the changes to `/etc/pulse/default.pa`.
    * Reboot the RDK board to reload PulseAudio configuration.
    * Alternatively, restart the PulseAudio service (if supported, e.g., `systemctl --user restart pulseaudio.service` or `pulseaudio -k && pulseaudio --start`), but a full reboot is usually cleaner.

After rebooting, you should see both input and output devices for the two sound cards in your system's sound settings (if using a desktop environment) or via `pactl list sources` / `pactl list sinks`, and be able to select and use them.
