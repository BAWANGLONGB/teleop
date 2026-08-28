# MarvinPlatform

## 你好使用软件前，需要温馨提示您：

    工程主目录下的CommonConfig/config文件夹内请放入独一无二的机型配置文件:
        ccs 6公斤的机型的有两个版本: 3.1(计算配置文件为ccs_m6_31.MvKDCfg), 4.0(计算配置文件为ccs_m6_40.MvKDCfg)，两个版本的参数不一样请确认版本后选择参数.
        ccs 3公斤的机型的计算配置文件为ccs_m3.MvKDCfg； 
        srs机型为srs.MvKDCfg. 多个*.MvKDCfg会解析出错.


## 安装APP:
    1. 我们测试并提供在WINDOWS 和UBUNTU20.04_X86下可执行的软件,如果与您的环境不一致,请下载源码后编译库,直接运行或者生成可执行APP运行
    2. MarvinPlatform基础环境:python3.10+, pyinstaller, pillow
    3. 运行前请确认:
            3.1. MarvinPlatform基础环境:python3.10及以上, pyinstaller,pillow
            3.2. 确保 SDK_PYTHON文件夹下为./contrlSDK100343  和 ./kinematicsSDK 下编译的最新的动态库SO: libMarvinSDK.so 和 libKine.so 
    4. 运行APP:
            4.1. 源码运行 ui_EN.py
            4.2. 生成可执行文件后运行,以便于分发到其他无PY环境的电脑上: python  setup.py

## 界面字体缩放

界面已固定为系统 DPI 的 150%，基础字体固定为 12pt，无需设置环境变量。主界面横向控件宽度按原布局压缩约 10%，STOP 按钮半径增大到 45。

```bash
# Linux 源码运行
python3 ui_EN.py

# Linux 打包程序（需先用修改后的源码重新打包）
./MarvinPlatform_linux_100343_large_ui
```

### 中文版

中文版与英文版共用同一套 SDK 和控制逻辑，仅翻译用户界面文字。
中文界面保留固定 150% DPI、12pt 字体、90% 横向间距和放大的急停按钮。

```bash
# Linux 源码运行
python3 ui_CN.py

# Linux 中文版可执行程序
./MarvinPlatform_linux_100343_CN

# 重新打包中文版
python3 setup_CN.py
```

Windows PowerShell：

```powershell
python ui_EN.py
```


## MARVIN APP使用说明文档

[MarvinPlatform软件使用说明](天机Marvin系列MarvinPlatform软件使用说明2601.pptx)
