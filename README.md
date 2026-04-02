# 基于Stm32F103的仿生蝴蝶🦋项目

## 快速开始
本工程为cmake,在编译之前确保你已配置好相关环境
- 配置arm工具链
- 配置下载器驱动,Jlink/OpenOcd
- 确保你已安装`cmake`和构建工具`Ninja`
- vscode下载相关插件

配置cmake
利用`Cmake Tool`插件配置或者命令:
```bash
camke --preset Debug 
```
之后即可利用`Crtl + Shift +B`一键编译

在,vscode/c_cpp_properties.json中将编译器路径修改为你自己电脑上的编译器路径即可解决vscode IntelliSense高亮报错问题
