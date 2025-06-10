---
sidebar_position: 1
---

# dpkg

Debian Linux系统上安装、创建和管理软件包。

**dpkg命令** 是Debian Linux系统用来安装、创建和管理软件包的实用工具。

## 语法说明

```
dpkg [`option` ...] `command`
```

## command 说明

dpkg 命令不仅有选项可以设置，还需要设置命令来执行不同的功能。

- -i：安装软件包；
- -r：删除软件包；
- -P：删除软件包的同时删除其配置文件；
- -L：列出属于指定软件包的文件；
- -l：简明地列出软件包的状态。
- -S：搜索含有指定文件的软件包。
- --unpack：解开软件包；
- -c：显示软件包内文件列表；
- --confiugre：配置软件包。

## 选项说明

- --admindir=`目录`          使用 `目录` 而非 /var/lib/dpkg。
- --root=`目录`              安装到另一个根目录下。
- --instdir=`目录`           改变安装目录的同时保持管理目录不变。
- --path-exclude=`表达式`    不要安装符合Shell表达式的路径。
- --path-include=`表达式`    在排除模式后再包含一个模式。
- -O|--selected-only         忽略没有被选中安装或升级的软件包。
- -E|--skip-same-version     忽略版本与已安装软件版本相同的软件包。
- -G|--refuse-downgrade      忽略版本早于已安装软件版本的的软件包。
- -B|--auto-deconfigure      就算会影响其他软件包，也要安装。
- --[no-]triggers            跳过或强制随之发生的触发器处理。
- --verify-format=`格式`     检查输出格式('rpm'被支持)。
- --no-debsig                不去尝试验证软件包的签名。
- -D|--debug=`八进制数`      开启调试(参见 -Dhelp 或者 --debug=help)。
- --status-logger=`命令`     发送状态更新到 `命令` 的标准输入。
- --log=`文件名`             将状态更新和操作信息到 `文件名`。
- --ignore-depends=`软件包`,... 		 忽略关于 `软件包` 的所有依赖关系。
- --force-...                忽视遇到的问题(参见 --force-help)。
- --no-force-...|--refuse-... 	 当遇到问题时中止运行。
- --abort-after ``n``         累计遇到 ``n`` 个错误后中止。

## 常用命令

- 安装包

```
dpkg -i package.deb
```

- 删除包

```
dpkg -r package
```

- 删除包（包括配置文件）

```
dpkg -P package
```

- 列出与该包关联的文件

```
dpkg -L package
```

- 显示该包的版本

```
dpkg -l package
```

- 解开deb包的内容

```
dpkg --unpack package.deb
```

- 搜索所属的包内容

```
dpkg -S keyword
```

- 列出当前已安装的包

```
dpkg -l
```

- 列出deb包的内容

```
dpkg -c package.deb
```

- 配置包

```
dpkg --configure package
```
