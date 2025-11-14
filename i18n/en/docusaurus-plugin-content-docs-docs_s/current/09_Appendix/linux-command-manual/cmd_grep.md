---
sidebar_position: 1
---

# grep command

Powerful text search tool.

**grep** (global search regular expression (RE) and print out the line) is a powerful text search tool that can search text using regular expressions and print out the matching lines. It is used to filter/search for specific characters. It can be used in combination with various commands, making it highly flexible.

Similar commands include egrep, fgrep, rgrep.

## Syntax

```
  grep [OPTION]... PATTERNS [FILE]...
  grep [OPTION...] PATTERNS [FILE...]
  grep [OPTION...] -e PATTERNS ... [FILE...]
  grep [OPTION...] -f PATTERN_FILE ... [FILE...]
```

- **PATTERNS** - Represents the string or regular expression to search for.
- **FILE** - Represents the file name to search in. Multiple files can be searched at once. If the `FILE` parameter is omitted, it defaults to reading data from standard input.

## Option Explanation

**Commonly used options:**

- `-i`: Ignore case for matching.
- `-v`: Print only non-matching lines.
- `-n`: Display line numbers of matching lines.
- `-r`: Recursively search files in subdirectories.
- `-l`: Print only the names of matching files.
- `-c`: Print only the count of matching lines.

**Additional parameter explanations:**



- **-a or --text**: Do not ignore binary data.
- **-A`<num\>` or --after-context=`<num\>`**: In addition to displaying the column that matches the pattern, display the lines after that line.
- **-b or --byte-offset**: Before displaying the line that matches the pattern, indicate the number of the first character in that line.
- **-B`<num\>` or --before-context=`<num\>`**: In addition to displaying the column that matches the pattern, display the lines before that line.
- **-c or --count**: Count the number of matching lines.
- **-C`<num\>` or --context=`<num\>` or -`<num\>`**: In addition to displaying the column that matches the pattern, display the lines before and after that line.
- **-d `<action\>` or --directories=`<action\>`**: When searching directories instead of files, this parameter must be used, otherwise the grep command will report an error and stop.
- **-e`<pattern\>` or --regexp=`<pattern\>`**: Specify a string as the pattern for searching file contents.
- **-E or --extended-regexp**: Use extended regular expressions as the pattern.
- **-f`<pattern_file\>` or --file=`<pattern_file\>`**: Specify a pattern file that contains one or more pattern styles for grep to search for in file contents, with one pattern style per line.
- **-F or --fixed-regexp**: Treat the pattern as a list of fixed strings.
- **-G or --basic-regexp**: Treat the pattern as a plain representation.
- **-h or --no-filename**: Do not display the file name before the line that matches the pattern.
- **-H or --with-filename**: Displays the file name before the line that matches the pattern.
- **-i or --ignore-case**: Ignores case differences in the pattern.
- **-l or --file-with-matches**: Lists the file names that contain matches of the specified pattern.
- **-L or --files-without-match**: Lists the file names that do not contain any matches of the specified pattern.
- **-n or --line-number**: Displays the line numbers before the lines that match the pattern.
- **-o or --only-matching**: Only displays the matching part of the pattern.
- **-q or --quiet or --silent**: Does not display any information.
- **-r or --recursive**: The same effect as specifying "-d recurse".
- **-s or --no-messages**: Does not display error messages.
- **-v or --invert-match**: Displays all lines that do not contain the matching text.
- **-V or --version**: Displays version information.
- **-w or --word-regexp**: Only displays whole words that match the pattern.
- **-x --line-regexp**: Only displays whole lines that match the pattern.
- **-y**: The same effect as specifying "-i".



**Regular expressions:**

```shell
^    # Anchors the match to the beginning of a line. Example: '^grep' matches any line that starts with 'grep'.
$    # Anchors the match to the end of a line. Example: 'grep$' matches any line that ends with 'grep'.
.    # Matches any single character except a newline character. Example: 'gr.p' matches 'gr' followed by any character, then 'p'.
*    # Matches zero or more of the preceding character. Example: '*grep' matches any line that has zero or more spaces followed by 'grep'.
.*   # Matches any sequence of characters.
[]   # Matches a single character within the specified range. Example: '[Gg]rep' matches 'Grep' or 'grep'.
[^]  # Matches a single character not within the specified range. Example: '[^A-Z]rep' matches any line that does not start with a letter between A and Z, followed by 'rep'.
\(..\)  # Marks a matching pattern. Example: '\(love\)', 'love' is marked as pattern 1.
\<      # Anchors the match to the beginning of a word. Example: '\<grep' matches any line with a word that starts with 'grep'.
\>      # Anchors the match to the end of a word. Example: 'grep\>' matches any line with a word that ends with 'grep'.
x\{m\}  # Repeats the character 'x' exactly 'm' times. Example: '0\{5\}' matches any line with exactly 5 'o's.
x\{m,\}   # Repeats the character 'x' at least 'm' times. Example: 'o\{5,\}' matches any line with at least 5 'o's.
x\{m,n\}  # Repeats the character 'x' at least 'm' times but no more than 'n' times. Example: 'o\{5,10\}' matches any line with 5 to 10 'o's.
\w    # Matches word and digit characters, equivalent to [A-Za-z0-9]. Example: 'G\w*p' matches a word that starts with 'G', followed by zero or more word or digit characters, and ends with 'p'.
\W    # Matches a non-word character. Example: '\W' matches any non-word character.
\b    # Matches a word boundary. Example: '\bgrep\b' only matches the word 'grep'.
```

## Common commands

Searches for a word in a file and returns the lines that contain the pattern **"match_pattern"**.

```shell
grep match_pattern file_name
grep "match_pattern" file_name
```

Searches for a pattern in multiple files.

```shell
grep "match_pattern" file_1 file_2 file_3 ...
```

Recursively search for lines matching the regular expression `pattern` in all files within the directory `dir`, and print the filenames and line numbers of the matching lines:

```
grep -r -n pattern dir/
```

Search for the string `world` in standard input and only print the number of matching lines:

```
echo "hello world" | grep -c world
```

In the current directory, search for files with the word "file" in their suffix that contain the string "test", and print the lines with that string. You can use the following command for this:

```
grep test *file
```

Search for files that meet the criteria recursively. For example, search for files in the specified directory `/etc/` and its subdirectories (if any) that contain the string `update`, and print the lines with that string. The command to use is:

```
grep -r update /etc
```

Perform a reverse search, using the `-v` parameter to print the contents of lines that do not match the criteria. Search for files with `conf` in their filenames that do not contain `test` on the lines. The command to use is:

```
grep -v test *conf*
```

Use the `--color=auto` option to mark the matching text with color:

```shell
grep "match_pattern" file_name --color=auto
```

Use the `-E` option to use regular expressions:

```shell
grep -E "[1-9]+"
# or
egrep "[1-9]+"
```

Use the `-P` option to use Perl-compatible regular expressions:

```shell
grep -P "(\d{3}\-){2}\d{4}" file_name
```

Only output the matching part in the file with the **-o** option.

```shell
echo this is a test line. | grep -o -E "[a-z]+\."
line.

echo this is a test line. | egrep -o "[a-z]+\."
line.
```

Count the number of lines in a file or text that contain the matching string with the **-c** option.

```shell
grep -c "text" file_name
```

Search for records in the command line history that have entered the `git` command.

```shell
history | grep git
```

Output the number of lines that contain the matching string with the **-n** option.

```shell
grep "text" -n file_name
# Or
cat file_name | grep "text" -n
# For multiple files
grep "text" -n file_1 file_2
```

Print the character or byte offset where the pattern match is located.

```shell
echo gun is not unix | grep -b -o "not"
# The character offset of the string in a line is calculated from the first character of that line, with the starting value as 0. The **-b -o** options are generally used together.
```

Search multiple files and find which files contain the matching text.

```shell
grep -l "text" file1 file2 file3...
```