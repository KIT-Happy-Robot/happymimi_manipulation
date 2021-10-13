# dynamixel_controller
## Overview
Dynamixelモータの制御プログラムやモジュールをまとめたパッケージ  

## Description
このパッケージが提供する機能は主に以下の4つです。
- 各Dynamixelモータの角度を出力するPublisher
- マニピュレータや首の制御
- 物体の受け渡しや設置
  
## Usage
### Motor Angle Publisher

  
**仕様**
| Communication | Name | Type |  |
| :---: | :---: | :---: | :---: |
| Topic | /servo/angle_list | Float64MultiArray |  |





| Module | Communication | Name | Type | Request | Result |
| :---: | :---: | :---: | :---: | :---: | :---: |
| Find | Service | /recognition/find | [RecognitionFind](https://github.com/KIT-Happy-Robot/happymimi_recognition/blob/master/happymimi_recognition_msgs/srv/RecognitionFind.srv) | string型: `target_name` | bool型: `result` |
  
**target_nameの種類**
| target_name | Contents |
| :---: | :--- |
| 特定の名前(ex. personとかcupとか) | 指定した名前の物体を探す |
| any | 把持可能物体を探す |
| (入力なし) | 何かしらの物体を探す |
  
---
