[English](./README.md) | 简体中文

欢迎使用本项目！此文档将帮助您快速开始安装、开发、构建和部署RDK_DOC的相关操作。


### 一、环境安装

安装本项目的依赖，请执行以下命令：

```shell
npm install

# 如安装失败
npm install --registry=https://registry.npmmirror.com
```

### 二、在线运行

仅构建中文手册：

```shell
npm run start
```

仅构建英文手册：

```shell
npm run start  -- --locale en
```

本方式无法实现文档的中英文切换，只能实现单种语言文档的构建，如果需要中英文同步显示，需要参考步骤三的方式。


### 三、离线部署

若需要完全离线部署手册，请运行以下脚本将图片下载到本地：

```shell
python3 download_imgs.py
```

文档编译确的编译部署方式，使用以下命令：

```shell
npm run build
```

文档部署方式，使用以下命令：

```shell
#直接部署

npm run serve

#指定ip地址和端口号部署

npm run serve -- --host=10.64.62.34 --port=1688 --no-open

```

启动一个静态文件服务器，并在浏览器中提供以下链接进行访问,端口号以实际端口号为主：

**英文手册链接**：http://localhost:3000/en/rdk_doc/

**中文手册链接**：http://localhost:3000/rdk_doc/


**注意：** 请确保使用的 Node.js 版本 为18.0或以上版本。

