---
sidebar_position: 7
---
# 8.7 Desktop Applications

This section primarily addresses issues encountered when using third-party applications on the desktop.

<!-- ```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
``` -->

### Q1: The downloaded Visual Studio Code application won't open?

<!-- <Tabs groupId="accessory">
<TabItem value="rdk_s600" label="rdk_s600"> -->

**A:**
* **Open via command line:** Visual Studio Code uses an Electron shell that has known issues with GPU (Graphics Processing Unit) hardware acceleration on certain systems. You can try disabling GPU acceleration by launching VS Code with the Electron `--disable-gpu` command-line flag (https://code.visualstudio.com/docs/supporting/faq#_vs-code-is-blank):

```bash
    code --disable-gpu
```
<!-- </TabItem>
</Tabs> -->