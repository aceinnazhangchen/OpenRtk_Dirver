npm install


如果编译错误，以管理员运行PowerShell，输入set-ExecutionPolicy RemoteSigned，按Y确认即可。
添加编译文件binding.gyp中添加文件，先运行node-gyp configure，再运行node-gyp build。