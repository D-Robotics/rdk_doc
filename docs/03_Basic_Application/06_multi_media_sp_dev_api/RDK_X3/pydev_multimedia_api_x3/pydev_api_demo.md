---
sidebar_position: 6
---

# 接口使用示例

## 单元测试
以下示例代码包含多个单元测试用例，覆盖了本章节接口的使用方式，仅作参考，具体如下：

```python
import sys, os, time

import numpy as np
import cv2
from hobot_vio import libsrcampy

def get_nalu_pos(byte_stream):
    size = byte_stream.__len__()
    nals = []
    retnals = []

    startCodePrefixShort = b"\x00\x00\x01"

    pos = 0
    while pos < size:
        is4bytes = False
        retpos = byte_stream.find(startCodePrefixShort, pos)
        if retpos == -1:
            break
        if byte_stream[retpos - 1] == 0:
            retpos -= 1
            is4bytes = True
        if is4bytes:
            pos = retpos + 4
        else:
            pos = retpos + 3
        val = hex(byte_stream[pos])
        val = "{:d}".format(byte_stream[pos], 4)
        val = int(val)
        fb = (val >> 7) & 0x1
        nri = (val >> 5) & 0x3
        type = val & 0x1f
        nals.append((pos, is4bytes, fb, nri, type))
    for i in range(0, len(nals) - 1):
        start = nals[i][0]
        if nals[i + 1][1]:
            end = nals[i + 1][0] - 5
        else:
            end = nals[i + 1][0] - 4
        retnals.append((start, end, nals[i][1], nals[i][2], nals[i][3], nals[i][4]))
    start = nals[-1][0]
    end = byte_stream.__len__() - 1
    retnals.append((start, end, nals[-1][1], nals[-1][2], nals[-1][3], nals[-1][4]))
    return retnals

def get_h264_nalu_type(byte_stream):
    nalu_types = []
    nalu_pos = get_nalu_pos(byte_stream)

    for idx, (start, end, is4bytes, fb, nri, type) in enumerate(nalu_pos):
        # print("NAL#%d: %d, %d, %d, %d, %d" % (idx, start, end, fb, nri, type))
        nalu_types.append(type)
    
    return nalu_types

def test_camera():
    cam = libsrcampy.Camera()
    ret = cam.open_cam(0, 1, 30, 1920, 1080)
    print("Camera open_cam return:%d" % ret)
    # wait for isp tuning
    time.sleep(1)
    img = cam.get_img(2)
    if img is not None:
        #save file
        fo = open("output.img", "wb")
        fo.write(img)
        fo.close()
        print("camera save img file success")
    else:
        print("camera save img file failed")
    cam.close_cam()
    print("test_camera done!!!")

def test_camera_vps():
    #vps start
    vps = libsrcampy.Camera()
    ret = vps.open_vps(1, 1, 1920, 1080, 512, 512)
    print("Camera vps return:%d" % ret)

    fin = open("output.img", "rb")
    img = fin.read()
    fin.close()
    ret = vps.set_img(img)
    print ("Process set_img return:%d" % ret)

    fo = open("output_vps.img", "wb+")
    img = vps.get_img()
    if img is not None:
        fo.write(img)
        print("encode write image success")
    else:
        print("encode write image failed")
    fo.close()

    vps.close_cam()
    print("test_camera_vps done!!!")

def test_encode():
    #encode file
    enc = libsrcampy.Encoder()
    ret = enc.encode(0, 1, 1920, 1080)
    print("Encoder encode return:%d" % ret)

    #save file
    fo = open("encode.h264", "wb+")
    a = 0
    fin = open("output.img", "rb")
    input_img = fin.read()
    fin.close()
    while a < 100:
        ret = enc.encode_file(input_img)
        print("Encoder encode_file return:%d" % ret)
        img = enc.get_img()
        if img is not None:
            fo.write(img)
            print("encode write image success count: %d" % a)
        else:
            print("encode write image failed count: %d" % a)
        a = a + 1

    enc.close()
    print("test_encode done!!!")

def test_decode():
    #decode start
    dec = libsrcampy.Decoder()

    ret = dec.decode("encode.h264", 0, 1, 1920, 1080)
    print ("Decoder return:%d frame count: %d" %(ret[0], ret[1]))

    img = dec.get_img()
    if img is not None:
        #save file
        fo = open("output.img", "wb")
        fo.write(img)
        fo.close()
        print("decode save img file success")
    else:
        print("decode save img file failed")

    dec.close()
    print("test_decode done!!!")
    
def test_display():
    disp = libsrcampy.Display()
    ret = disp.display(0, 1920, 1080, 0, 1)
    print ("Display display 0 return:%d" % ret)
    ret = disp.display(2)
    print ("Display display 2 return:%d" % ret)
    ret = disp.set_graph_rect(100, 100, 1920, 200, chn = 2, flush = 1,  color = 0xffff00ff)
    print ("Display set_graph_rect return:%d" % ret)
    string = "horizon"
    ret = disp.set_graph_word(300, 300, string.encode('gb2312'), 2, 0, 0xff00ffff)
    print ("Display set_graph_word return:%d" % ret)
    
    fo = open("output.img", "rb")
    img = fo.read()
    fo.close()
    ret = disp.set_img(img)
    print ("Display set_img return:%d" % ret)

    time.sleep(3)

    disp.close()
    print("test_display done!!!")

def test_camera_bind_encode():
    #camera start
    cam = libsrcampy.Camera()
    ret = cam.open_cam(0, 1, 30, [1920, 1280], [1080, 720])
    print("Camera open_cam return:%d" % ret)

    #encode start
    enc = libsrcampy.Encoder()
    ret = enc.encode(0, 1, 1920, 1080)
    print("Encoder encode return:%d" % ret)
    ret = libsrcampy.bind(cam, enc)
    print("libsrcampy bind return:%d" % ret)

    enc1 = libsrcampy.Encoder()
    ret = enc1.encode(1, 1, 1280, 720)
    print("Encoder encode return:%d" % ret)
    ret = libsrcampy.bind(cam, enc1)
    print("libsrcampy bind return:%d" % ret)

    #save file
    fo = open("encode.h264", "wb+")
    fo1 = open("encode1.h264", "wb+")
    a = 0
    while a < 100:
        img = enc.get_img()
        img1 = enc1.get_img()
        if img is not None:
            fo.write(img)
            fo1.write(img1)
            print("encode write image success count: %d" % a)
        else:
            print("encode write image failed count: %d" % a)
        a = a + 1
    fo.close()
    fo1.close()

    print("save encode file success")
    ret = libsrcampy.unbind(cam, enc)
    print("libsrcampy unbind return:%d" % ret)
    ret = libsrcampy.unbind(cam, enc1)
    print("libsrcampy unbind return:%d" % ret)

    enc1.close()
    enc.close()
    cam.close_cam()
    print("test_camera_bind_encode done!!!")

def test_camera_bind_display():
    #camera start
    cam = libsrcampy.Camera()
    ret = cam.open_cam(0, 1, 30, 1280, 720)
    print("Camera open_cam return:%d" % ret)

    #display start
    disp = libsrcampy.Display()
    ret = disp.display(0, 1920, 1080, 0, 1, chn_width = 1280, chn_height = 720)
    print ("Display display 0 return:%d" % ret)
    ret = disp.display(2, chn_width = 1280, chn_height = 720)
    print ("Display display 2 return:%d" % ret)
    disp.set_graph_rect(100, 100, 1920, 200, chn = 2, flush = 1,  color = 0xffff00ff)
    string = "horizon"
    disp.set_graph_word(300, 300, string.encode('gb2312'), 2, 0, 0xff00ffff)
    ret = libsrcampy.bind(cam, disp)
    print("libsrcampy bind return:%d" % ret)
    
    time.sleep(10)

    ret = libsrcampy.unbind(cam, disp)
    print("libsrcampy unbind return:%d" % ret)
    disp.close()
    cam.close_cam()
    print("test_camera_bind_display done!!!")

def test_decode_bind_display():
    #decode start
    dec = libsrcampy.Decoder()
    ret = dec.decode("encode.h264", 0, 1, 1920, 1080)
    print ("Decoder return:%d frame count: %d" %(ret[0], ret[1]))

    dec1 = libsrcampy.Decoder()
    ret = dec1.decode("encode1.h264", 1, 1, 1280, 720)
    print ("Decoder return:%d frame count: %d" %(ret[0], ret[1]))

    #display start
    disp = libsrcampy.Display()
    ret = disp.display(0, 1920, 1080, 0, 1)
    print ("Display display 0 return:%d" % ret)
    ret = disp.display(2)
    print ("Display display 2 return:%d" % ret)
    disp.set_graph_rect(100, 100, 1920, 200, chn = 2, flush = 1,  color = 0xffff00ff)
    string = "horizon"
    disp.set_graph_word(300, 300, string.encode('gb2312'), 2, 0, 0xff00ffff)
    ret = libsrcampy.bind(dec, disp)
    print("libsrcampy bind return:%d" % ret)
    
    time.sleep(5)

    ret = libsrcampy.unbind(dec, disp)
    print("libsrcampy unbind return:%d" % ret)
    disp.close()
    dec1.close()
    dec.close()
    print("test_decode_bind_display done!!!")

def test_cam_bind_encode_decode_bind_display():
    #camera start
    cam = libsrcampy.Camera()
    ret = cam.open_cam(0, 1, 30, [1920, 1280], [1080, 720])
    print("Camera open_cam return:%d" % ret)

    #encode file
    enc = libsrcampy.Encoder()
    ret = enc.encode(0, 1, 1920, 1080)
    print("Encoder encode return:%d" % ret)

    #decode start
    dec = libsrcampy.Decoder()
    ret = dec.decode("", 0, 1, 1920, 1080)
    print ("Decoder return:%d frame count: %d" %(ret[0], ret[1]))

    #display start
    disp = libsrcampy.Display()
    ret = disp.display(0, 1920, 1080, 0, 1)
    print ("Display display 0 return:%d" % ret)

    ret = libsrcampy.bind(cam, enc)
    print("libsrcampy bind return:%d" % ret)
    ret = libsrcampy.bind(dec, disp)
    print("libsrcampy bind return:%d" % ret)

    a = 0
    while a < 100:
        img = enc.get_img()
        if img is not None:
            dec.set_img(img)
            print("encode get image success count: %d" % a)
        else:
            print("encode get image failed count: %d" % a)
        a = a + 1

    ret = libsrcampy.unbind(cam, enc)
    ret = libsrcampy.unbind(dec, disp)
    disp.close()
    dec.close()
    enc.close()
    cam.close_cam()
    print("test_cam_bind_encode_decode_bind_display done!!!")

def test_cam_vps_display():
    #camera start
    cam = libsrcampy.Camera()
    ret = cam.open_cam(0, 1, 30, [1920, 1280], [1080, 720])
    print("Camera open_cam return:%d" % ret)

    #vps start
    vps = libsrcampy.Camera()
    ret = vps.open_vps(1, 1, 1920, 1080, 512, 512)
    print("Camera vps return:%d" % ret)

    #display start
    disp = libsrcampy.Display()
    ret = disp.display(0, 1920, 1080, 0, 1)
    print ("Display display 0 return:%d" % ret)

    a = 0
    while a < 100:
        img = cam.get_img()
        if img is not None:
            vps.set_img(img)
            print("camera get image success count: %d" % a)
        else:
            print("camera get image failed count: %d" % a)

        img = vps.get_img(2, 1920, 1080)
        if img is not None:
            disp.set_img(img)
            print("vps get image success count: %d" % a)
        else:
            print("vps get image failed count: %d" % a)
        a = a + 1

    disp.close()
    vps.close_cam()
    cam.close_cam()
    print("test_cam_vps_display done!!!")

def test_rtsp_decode_bind_vps_bind_disp(rtsp_url):
    start_time = time.time()
    image_count = 0
    skip_count = 0
    find_pps_sps = 0

    #rtsp start
    cap = cv2.VideoCapture(rtsp_url)
    cap.set(cv2.CAP_PROP_FORMAT, -1) # get stream
    if not cap.isOpened():
        print("fail to open rtsp: {}".format(rtsp_url))
        return -1
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    #decode start
    dec = libsrcampy.Decoder()
    ret = dec.decode("", 0, 1, width, height)
    print ("Decoder return:%d frame count: %d" %(ret[0], ret[1]))

    #camera start
    vps = libsrcampy.Camera()
    ret = vps.open_vps(0, 1, width, height, [1920, 512], [1080, 512])
    print("Camera open_cam return:%d" % ret)

    #display start
    disp = libsrcampy.Display()
    ret = disp.display(0, 1920, 1080, 0, 1)
    print ("Display display 0 return:%d" % ret)

    ret = libsrcampy.bind(dec, vps)
    print("libsrcampy bind return:%d" % ret)
    ret = libsrcampy.bind(vps, disp)
    print("libsrcampy bind return:%d" % ret)

    a = 0
    while True:
        ret, stream_frame = cap.read()
        if not ret:
            return
        nalu_types = get_h264_nalu_type(stream_frame.tobytes())

        # 送入解码的第一帧需要是 pps，sps, 否则解码器会报 "FAILED TO DEC_PIC_HDR" 异常而退出
        if (nalu_types[0] in [1, 5]) and find_pps_sps == 0:
            continue

        find_pps_sps = 1
        if stream_frame is not None:
            ret = dec.set_img(stream_frame.tobytes(), 0) # 发送码流, 先解码数帧图像后再获取
            if ret != 0:
                return ret
            if skip_count < 5:
                skip_count += 1
                image_count = 0
                continue

    ret = libsrcampy.unbind(dec, vps)
    ret = libsrcampy.unbind(vps, disp)
    disp.close()
    dec.close()
    vps.close_cam()
    cap.release()
    print("test_rtsp_decode_bind_vps_bind_disp done!!!")


test_camera()
test_camera_vps()
test_encode()
test_decode()
test_display()
test_camera_bind_encode()
test_camera_bind_display()
test_decode_bind_display()
test_cam_bind_encode_decode_bind_display()
test_cam_vps_display()

# rtsp_url = "rtsp://127.0.0.1/3840x2160.264"
# test_rtsp_decode_bind_vps_bind_disp(rtsp_url)
```


本章节将通过视频流解码等示例程序，介绍 D-Robotics Python 语言的 `hobot_vio` 图像多媒体库的使用方法，包括视频拉流、缩放及编解码等操作。

## 视频流解码

本示例代码位于`/app/pydev_demo/08_decode_rtsp_stream/` 目录下，所实现的功能有：
1. 通过 opencv 打开 rtsp 码流，获取到码流数据
2. 调用视频解码接口对码流进行解码
3. 把解码后的视频通过 HDMI 显示

### 运行方法

本示例运行依赖 rtsp 流，如用户不方便搭建 rtsp 推流服务，可使用系统预置的推流服务。该服务会把`1080P_test.h264`视频文件处理成 rtsp 流，url 地址为`rtsp://127.0.0.1/1080P_test.h264`。

用户可通过如下命令启动推流服务：

```
cd /app/pydev_demo/08_decode_rtsp_stream/
root@ubuntu:/app/pydev_demo/08_decode_rtsp_stream# sudo ./live555MediaServer &
```

服务正常启动后的log如下， 注意最后一行的 `We use port 80`, 说明rtsp服务运行在80端口，它有可能存在8000和8080的情况，在后面设置rtsp url的时候需要根据实际使用的端口号做修改：
```bash
root@ubuntu:/app/pydev_demo/08_decode_rtsp_stream#
LIVE555 Media Server version 1.01 (LIVE555 Streaming Media library version 2020.07.09).
Play streams from this server using the URL
        rtsp://192.168.127.10/<filename>
where <filename> is a file present in the current directory.
Each file's type is inferred from its name suffix:
        ".264" => a H.264 Video Elementary Stream file
... 省略 ...
(We use port 80 for optional RTSP-over-HTTP tunneling, or for HTTP live streaming (for indexed Transport Stream files only).)
```

然后调用 `./decode_rtsp_stream.py ` 命令，启动拉流解码程序，并将 url 地址、分辨率、帧率等信息通过控制台输出，log 如下：

```shell
root@ubuntu:/app/pydev_demo/08_decode_rtsp_stream# ./decode_rtsp_stream.py
['rtsp://127.0.0.1/1080P_test.h264']
RTSP stream frame_width:1920, frame_height:1080
Decoder(0, 1) return:0 frame count: 0
Camera vps return:0
Decode CHAN: 0 FPS: 30.34
Display FPS: 31.46
Decode CHAN: 0 FPS: 25.00
Display FPS: 24.98
RTSP stream frame_width:1920, frame_height:1080
```

最后，视频流会通过 HDMI 接口输出，用户可以通过显示器预览视频画面。

### 选项参数说明

示例程序`decode_rtsp_stream.py`可通过修改启动参数，设置rtsp地址、开关HDMI输出、开关推理等功能。参数说明如下：

- **-u**  ： 设置rtsp网络地址，支持输入多个地址，如：`-u "rtsp://127.0.0.1/1080P_test.h264;rtsp://192.168.1.10:8000/1080P_test.h264"`
- **-d**  ： 开启、关闭HDMI的显示输出，不设置时默认开启显示，`-d 0 ` 关闭显示，多路解码时只显示第一路的视频
- **-a**  ： 开启、关闭算法推理功能，不设置时默认关闭算法，`-a`开启算法推理，运行目标检测算法

**几种常用的启动方式**

解码默认流并开启HDMI显示
```
sudo ./decode_rtsp_stream.py
```
解码默认流并关闭HDMI显示
```
sudo ./decode_rtsp_stream.py -d 0
```
解码单路rtsp流
```
sudo ./decode_rtsp_stream.py -u "rtsp://x.x.x.x/xxx"
```
解码多路rtsp流
```
sudo ./decode_rtsp_stream.py -u "rtsp://x.x.x.x/xxx;rtsp://x.x.x.x/xxx"
```
解码默认流并使能推理
```
sudo ./decode_rtsp_stream.py -a
```

### 注意事项

- 推流服务器推送的rtsp码流里面需要包含`PPS`和`SPS`参数信息，否则会导致开发板解码异常，错误信息如下：
![image-20220728110439753](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/08_FAQ/image/multimedia/image-20220728110439753.png)

- 使用`ffmpeg`打开`.mp4 .avi`等格式的视频文件推流时，需要添加`-vbsf h264_mp4toannexb`选项，以添加码流的`PPS` 和`SPS`信息，例如：

    ```
    ffmpeg -re -stream_loop -1 -i xxx.mp4 -vcodec copy -vbsf h264_mp4toannexb -f rtsp rtsp://192.168.1.195:8554/h264_stream
    ```

- rtsp视频流目前仅支持 1080P 分辨率

- 不支持使用 vlc 软件进行 rtsp 推流，原因是 vlc 软件不支持添加`PPS`和`SPS`信息
