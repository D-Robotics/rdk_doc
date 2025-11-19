---
sidebar_position: 4
---
# PCIe User-space High Level API Introduction

## Introduction

The High Level API (`libhbpciehl.so`) is built upon the Low Level API (`libhbpcie.so`), abstracting generic topic/subscribe/publish concepts and hiding hardware differences across various D-Robotics chip series. This enables users to utilize PCIe for data communication more conveniently.

Key supported features include:

- Data transmission (publish)
- Data reception (subscribe)
- Management of send/receive buffers
  - Using built-in buffers
  - Using user-allocated buffers (requiring physically contiguous memory)

## API List

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

## Usage Workflow

The workflows for sender and receiver are illustrated below:

![Image Description](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/pcie/hl_process.png)

### Publisher (chip#0)

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

### Subscriber (chip#1)

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