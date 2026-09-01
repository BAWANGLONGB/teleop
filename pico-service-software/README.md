# PICO / PC Service 安装文件

本目录只保存安装文件的版本、校验值和获取方式。`.deb`、`.apk` 被本目录的
`.gitignore` 排除，避免把大二进制误提交到普通 Git 仓库；缺少本地文件时按下面
的官方地址下载即可。

| 文件 | 版本 / 平台 | SHA-256 |
| --- | --- | --- |
| `XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb` | PC Service 1.0.0，Ubuntu 22.04 x86_64 | `61961067eb4b41f81ed7cae35f4690dbb0ddfefb329a12b24e0b90ebc46ada91` |
| `XRoboToolkit-PICO-1.1.1.apk` | XRoboToolkit PICO 1.1.1 | `6b2bb282405673d24abcb1980e3478b8f1052e90f7207b1f24cc56a59f8d8261` |

## 官方来源

- [XR-Robotics 官方入口与 Get Started](https://github.com/XR-Robotics)：按页面说明获取
  `XRoboToolkit-PICO-1.1.1.apk`。
- [XRoboToolkit Unity Client v1.1.1 Release](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases/tag/v1.1.1)：
  PICO APK 发布页。
- PICO APK 直链：
  <https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases/download/v1.1.1/XRoboToolkit-PICO-1.1.1.apk>
- [XRoboToolkit PC Service v1.0.0 Release](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/tag/v1.0.0)：
  PC Service 发布页。
- PC Service DEB 直链：
  <https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb>

## 下载与校验

在仓库根目录执行。两个文件都可以从官方 Release 直链下载，并保持文件名不变。

```bash
mkdir -p pico-service-software
curl -fL \
  -o pico-service-software/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb \
  'https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb'

curl -fL \
  -o pico-service-software/XRoboToolkit-PICO-1.1.1.apk \
  'https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases/download/v1.1.1/XRoboToolkit-PICO-1.1.1.apk'

sha256sum -c pico-service-software/SHA256SUMS
```

如果官方发布页更换了资产位置，以官方发布页为准；下载后的 SHA-256 必须与
`SHA256SUMS` 一致。若需要把二进制也托管在 Git，应使用 GitHub Release asset 或
Git LFS，并由仓库维护者配置凭据；本项目不把安装包复制进普通 Git 历史。
