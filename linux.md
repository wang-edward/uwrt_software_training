# Introduction to Linux

In order to use linux, these ideas are very important to understand beforehand. Most of file navigation in linux is done through the terminal


#Understanding the Terminal
The terminal is accessed using `ctrl` + `alt` + `t`. Upon opening the terminal the user sees a shell to type in commands and a directory that indicates where the user is in their current file system

#Man Pages

Each of the commands introduced below have a cooresponding manual highlighting how to use it called the man page. The commands below are briefly mentioned with the intention of the reader to go through the man page to practice using it.



#Commands
This is a short list of commands to know. There is no need to memorize them.

- `man`
Gives the man page for a command.
- `ls`
Shows the contents of the current location.
- `cd`
Enter into a directory.
- `touch`
Create a file in the current directory.
- `mkdir`
Makes a folder at the current location.
- `mv`
Moves a file to a new location. Can also be used to rename a file.
- `cp`
Copies a file to a new location.
- `find`
Find a file.
- `source`
Run a script and keep the environment variables in the current terminal instance.

#Unique Directories
- `.`
Indicates the current directory
- `..`
Indicates the parent directory
- `~`
Indicates the root directory

#Examples

Example 1: Creating a file named `note.txt` 

```
> touch note.txt
```

Example 2: Finding a file named `note.txt` in my child folder called `src`

```
//File Structure
Parent --
        |
        ---- src --
                  |
                  ---- note.txt
                  ---- folder


```

```
> find ./src/note.txt 
```
Example 3: Entering src first. Then moving a file named `note.txt` from src to my parent directory
```
//File Structure
Parent --
        |
        ---- src --
                  |
                  ---- note.txt
                  ---- folder

```

```
> cd src
```
```
//File Structure

---- note.txt
---- folder
```

```
> mv ./note.txt ..
```

```
//File Structure
Parent --
        |
        ---- src --
                  |
                  ---- folder
        ---- note.txt
```

Example 4: Copying `note.txt` to my root 
```
//File Directory
~ --
   |
    ----Parent --
                |
                ---- src --
                          |
                           ---- note.txt
                           ---- folder
    
```
```
> cd ~/Parent/src
```

```
//File Directory
---- note.txt
---- folder
```
```
> cp note.txt ~
```
```
//File Directory
~ --
   |
   ---- note.txt
   ---- Parent --
                |
                ---- src --
                          |
                           ---- note.txt
                           ---- folder
   --
```