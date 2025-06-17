---
sidebar_position: 1
---
# grep

强大的文本搜索工具。

**grep** （global search regular expression(RE) and print out the line，全面搜索正则表达式并把行打印出来）是一种强大的文本搜索工具，它能使用正则表达式搜索文本，并把匹配的行打印出来。用于过滤/搜索的特定字符。可使用正则表达式能配合多种命令使用，使用上十分灵活。

同类命令还有egrep, fgrep, rgrep。

## 语法说明

```
  grep [OPTION]... PATTERNS [FILE]...
  grep [OPTION...] PATTERNS [FILE...]
  grep [OPTION...] -e PATTERNS ... [FILE...]
  grep [OPTION...] -f PATTERN_FILE ... [FILE...]
```

- **PATTERNS** - 表示要查找的字符串或正则表达式。
- **FILE** - 表示要查找的文件名，可以同时查找多个文件，如果省略 `FILE`参数，则默认从标准输入中读取数据。

## 选项说明

**常用选项说明：**

- `-i`：忽略大小写进行匹配。
- `-v`：反向查找，只打印不匹配的行。
- `-n`：显示匹配行的行号。
- `-r`：递归查找子目录中的文件。
- `-l`：只打印匹配的文件名。
- `-c`：只打印匹配的行数。

**更多参数说明**：

- **-a 或 --text** : 不要忽略二进制的数据。
- **-A`显示行数` 或 --after-context=`显示行数`** : 除了显示符合范本样式的那一列之外，并显示该行之后的内容。
- **-b 或 --byte-offset** : 在显示符合样式的那一行之前，标示出该行第一个字符的编号。
- **-B`显示行数` 或 --before-context=`显示行数`** : 除了显示符合样式的那一行之外，并显示该行之前的内容。
- **-c 或 --count** : 计算符合样式的列数。
- **-C`显示行数` 或 --context=`显示行数`或-`显示行数`** : 除了显示符合样式的那一行之外，并显示该行之前后的内容。
- **-d `动作` 或 --directories=`动作`** : 当指定要查找的是目录而非文件时，必须使用这项参数，否则grep指令将回报信息并停止动作。
- **-e`范本样式` 或 --regexp=`范本样式`** : 指定字符串做为查找文件内容的样式。
- **-E 或 --extended-regexp** : 将样式为延伸的正则表达式来使用。
- **-f`规则文件` 或 --file=`规则文件`** : 指定规则文件，其内容含有一个或多个规则样式，让grep查找符合规则条件的文件内容，格式为每行一个规则样式。
- **-F 或 --fixed-regexp** : 将样式视为固定字符串的列表。
- **-G 或 --basic-regexp** : 将样式视为普通的表示法来使用。
- **-h 或 --no-filename** : 在显示符合样式的那一行之前，不标示该行所属的文件名称。
- **-H 或 --with-filename** : 在显示符合样式的那一行之前，表示该行所属的文件名称。
- **-i 或 --ignore-case** : 忽略字符大小写的差别。
- **-l 或 --file-with-matches** : 列出文件内容符合指定的样式的文件名称。
- **-L 或 --files-without-match** : 列出文件内容不符合指定的样式的文件名称。
- **-n 或 --line-number** : 在显示符合样式的那一行之前，标示出该行的列数编号。
- **-o 或 --only-matching** : 只显示匹配PATTERN 部分。
- **-q 或 --quiet或--silent** : 不显示任何信息。
- **-r 或 --recursive** : 此参数的效果和指定"-d recurse"参数相同。
- **-s 或 --no-messages** : 不显示错误信息。
- **-v 或 --invert-match** : 显示不包含匹配文本的所有行。
- **-V 或 --version** : 显示版本信息。
- **-w 或 --word-regexp** : 只显示全字符合的列。
- **-x --line-regexp** : 只显示全列符合的列。
- **-y** : 此参数的效果和指定"-i"参数相同。

**规则表达式：**

```shell
^    # 锚定行的开始 如：'^grep'匹配所有以grep开头的行。  
$    # 锚定行的结束 如：'grep$' 匹配所有以grep结尾的行。
.    # 匹配一个非换行符的字符 如：'gr.p'匹配gr后接一个任意字符，然后是p。  
*    # 匹配零个或多个先前字符 如：'*grep'匹配所有一个或多个空格后紧跟grep的行。  
.*   # 一起用代表任意字符。   
[]   # 匹配一个指定范围内的字符，如'[Gg]rep'匹配Grep和grep。  
[^]  # 匹配一个不在指定范围内的字符，如：'[^A-Z]rep' 匹配不包含 A-Z 中的字母开头，紧跟 rep 的行
\(..\)  # 标记匹配字符，如'\(love\)'，love被标记为1。  
\<      # 锚定单词的开始，如:'\<grep'匹配包含以grep开头的单词的行。  
\>      # 锚定单词的结束，如'grep\>'匹配包含以grep结尾的单词的行。 
x\{m\}  # 重复字符x，m次，如：'0\{5\}'匹配包含5个o的行。  
x\{m,\}   # 重复字符x,至少m次，如：'o\{5,\}'匹配至少有5个o的行。  
x\{m,n\}  # 重复字符x，至少m次，不多于n次，如：'o\{5,10\}'匹配5--10个o的行。   
\w    # 匹配文字和数字字符，也就是[A-Za-z0-9]，如：'G\w*p'匹配以G后跟零个或多个文字或数字字符，然后是p。   
\W    # \w的反置形式，匹配一个或多个非单词字符，如点号句号等。   
\b    # 单词锁定符，如: '\bgrep\b'只匹配grep。  
```

## 常用命令

在文件中搜索一个单词，命令会返回一个包含 **“match_pattern”** 的文本行

```shell
grep match_pattern file_name
grep "match_pattern" file_name
```

在多个文件中查找

```shell
grep "match_pattern" file_1 file_2 file_3 ...
```

在文件夹 dir 中递归查找所有文件中匹配正则表达式 `pattern` 的行，并打印匹配行所在的文件名和行号

```
grep -r -n pattern dir/
```

在标准输入中查找字符串 `world`，并只打印匹配的行数

```
echo "hello world" | grep -c world
```

在当前目录中，查找后缀有 file 字样的文件中包含 `test`字符串的文件，并打印出该字符串的行。此时，可以使用如下命令

```
grep test *file 
```

以递归的方式查找符合条件的文件。例如，查找指定目录/etc/ 及其子目录（如果存在子目录的话）下所有文件中包含字符串 `update`的文件，并打印出该字符串所在行的内容，使用的命令为：

```
grep -r update /etc 
```

反向查找，通过 `-v`参数可以打印出不符合条件行的内容。查找文件名中包含 `conf` 的文件中不包含 `test`的行，此时，使用的命令为：

```
grep -v test *conf*
```

标记匹配颜色 **--color=auto** 选项

```shell
grep "match_pattern" file_name --color=auto
```

使用正则表达式 **-E** 选项

```shell
grep -E "[1-9]+"
# 或
egrep "[1-9]+"
```

使用正则表达式 **-P** 选项

```shell
grep -P "(\d{3}\-){2}\d{4}" file_name
```

只输出文件中匹配到的部分 **-o** 选项

```shell
echo this is a test line. | grep -o -E "[a-z]+\."
line.

echo this is a test line. | egrep -o "[a-z]+\."
line.
```

统计文件或者文本中包含匹配字符串的行数 **-c** 选项

```shell
grep -c "text" file_name
```

搜索命令行历史记录中 输入过 `git` 命令的记录

```shell
history | grep git
```

输出包含匹配字符串的行数 **-n** 选项

```shell
grep "text" -n file_name
# 或
cat file_name | grep "text" -n
#多个文件
grep "text" -n file_1 file_2
```

打印样式匹配所位于的字符或字节偏移

```shell
echo gun is not unix | grep -b -o "not"
#一行中字符串的字符偏移是从该行的第一个字符开始计算，起始值为0。选项  **-b -o**  一般总是配合使用。
```

搜索多个文件并查找匹配文本在哪些文件中

```shell
grep -l "text" file1 file2 file3...
```
