[English](./README.md) | 简体中文

欢迎使用本项目！此文档将帮助您快速开始安装、开发、构建和部署RDK_DOC的相关操作。


### 一、环境安装

安装本项目的依赖，请执行以下命令：

```shell
$ npm install
```

### 二、在线运行

仅构建中文手册：

```shell
$ npm run start
```

仅构建英文手册：

```shell
$ npm run start  -- --locale en
```

本方式无法实现文档的中英文切换，只能实现单种语言文档的构建，如果需要中英文同步显示，需要参考步骤三的方式。

或者开两个窗口，同时运行`npm run start`和`npm run start -- --locale en`



### 三、离线部署

文档编译确的编译部署方式，使用以下命令：

```shell
$ npm run build
```

文档部署方式，使用以下命令：

```shell
$ npm run serve
```

启动一个静态文件服务器，并在浏览器中提供以下链接进行访问,端口号以实际端口号为主：

**英文手册链接**：http://localhost:3000/en/docs/

**中文手册链接**：http://localhost:3000/docs/


