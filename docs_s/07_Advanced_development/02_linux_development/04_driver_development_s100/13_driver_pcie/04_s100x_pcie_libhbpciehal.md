---
sidebar_position: 4
---
# PCIe 用户态 High Level API介绍

## 简介

High Level API (libhbpciehl.so) 基于 Low Level API（libhbpcie.so)，抽象出通用的 topic / subscribe / publish 概念，屏蔽不同系列地瓜芯片的硬件差异，让用户能够更加便捷的使用 PCIe 进行数据通信。

主要支持如下功能：

- 数据的发送（publish）
- 数据的接收（subscribe）
- 发送/接收Buffer的管理
  - 使用内建buffer
  - 使用用户申请的buffer(要求物理地址连续）

## API列表

```c
pcieErrCode pcieInit(pcieHandler *ph, uint8_t chipID, uint8_t topicID);
pcieErrCode pcieDeInit(pcieHandler ph);
pcieErrCode pcieGetMaxTopicSize(pcieHandler ph, uint8_t *topicSize);
pcieErrCode pciePublish(pcieHandler ph, uint8_t weight);
pcieErrCode pcieSubscribe(pcieHandler ph);
pcieErrCode pcieGetMaxInnerBufSize(pcieHandler ph, uint32_t *size);
pcieErrCode pcieAllocInnerBuf(pcieHandler ph, uint32_t size, void **virtualAddr, uint64_t *physAddr);
pcieErrCode pcieRegisterUserBuf(pcieHandler ph, uint64_t physAddr, uint32_t size);
pcieErrCode pcieStartRecv(pcieHandler ph, recvDataCallBack fun, void *funData);
pcieErrCode pcieSendData(pcieHandler ph, uint32_t size);
```

## 使用流程

发送方和接收方的流程如下图：

![图片描述](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/pcie/hl_process.png)

### Publisher(chip#0)

```c
void main()
{
    uint32_t size;
    void *addr;
    uint64_t phys;
    pcieHandler ph;
    void *UserBuffer;
    uint64_t UserBufferPhys;

    /* connect chip1 topic0 */
    pcieInit(&ph, 1, 0);

    pciePublish(ph);

    if (useInnerBuffer) {
        pcieGetMaxInnerBufSize(ph, &size);
        pcieAllocInnerBuf(ph, &addr, &phys, size);
        /* fill user data to inner buffer */
        ...

    } else {
        /* prepare the User data */
        ...
        /* use data in user buffer */
        pcieRegisterUserBuf(ph, UserBufferPhys, size);
    }

    pcieSendData(ph, size);

    pcieDeInit(ph);

    return;
}
```

### Subscriber(chip#1)

```c
void recvDataHandler(pcieHandler ph, uint32_t RecvSize, void *pData)
{
    /* deal with the received data */
    ...
}

void main()
{
    void *pData;
    pcieHandler ph;
    uint32_t size;
    void *addr;
    uint64_t phys;

    /* connect chip0 topic0 */
    pcieInit(&ph, 0, 0);

    pcieSubscribe(ph);

    if (useInnerBuffer) {
        pcieGetMaxInnerBufSize(ph, &size);
        pcieAllocInnerBuf(ph, &addr, &phys, size);
    } else {
        /* alloc user buff */
        ...
        pcieRegisterUserBuf(ph, UserBufferPhys, size);
    }

    pcieStartRecv(ph, recvDataHandler, pData);

    /* wait for receiving data */
    while (1) {
        sleep(1);
    }

    pcieDeInit(ph);

    return;
}
```
