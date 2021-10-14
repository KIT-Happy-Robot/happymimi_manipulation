# dynamixel_controller
## Overview
Dynamixelモータの制御プログラムやモジュールをまとめたパッケージ  

## Description
このパッケージが提供する機能は主に以下の4つです。
- 各Dynamixelモータの角度を出力するパブリッシャ
- マニピュレータや首の制御
- 物体の受け渡しや設置
  
## Usage
### Motor Angle Publisher
mimiに搭載されている各Dynamixelモータの角度を出力するパブリッシャ  
  
**仕様**
| Communication | Name | Type | Output |
| :---: | :---: | :---: | :---: |
| Topic | /servo/angle_list | [Float64MultiArray](http://docs.ros.org/en/api/std_msgs/html/msg/Float64MultiArray.html) | float64[]型: `data` |
  
各モータはナンバリングされており、以下の表の様になっています。  
| Joint | Number |
| :--- | --- |
| 左肩 | 0 |
| 右肩 | 1 |
| 肘 | 2 |
| 手首 | 3 |
| 手先(エンドエフェクタ) | 4 |
| 首 | 5 |
  
このナンバリングは配列のインデックスと対応しており、  
(ex. data[5] -> 首の角度)のように角度を取得することができます。  
単位はdeg  

---
### Control Head Subscriber  
首モータの角度を変更するサブスクライバ  
  
**仕様**  
マイナスで下方向、プラスで上方向を向きます  
| Communication | Name | Type | Input |
| :---: | :---: | :---: | :---: |
| Topic | /servo/head | [Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html) | float64型: `data` |
  
**注意**  
`data`には-30～40[deg]の範囲で入力してください。  

---
### Change Arm Pose
マニピュレータを用いた応用型モジュール  
物体の受け渡しや物体の設置などができます。  
  
| Communication | Name | Type | Request | Result |
| :---: | :---: | :---: | :---: | :---: |
| Service | /servo/arm | [StrTrg](https://github.com/KIT-Happy-Robot/happymimi_robot/blob/develop/happymimi_msgs/srv/StrTrg.srv) | string型: `data` | bool型: `result` |
  
**dataの種類**
| data | Contents |
| :---: | :--- |
| origin | アームを伸ばし、地面と水平にする |
| carry | アームを折りたたみ、コンパクトにする |
| receive | 物体を受け取る, 終了後、carryに |
| give | 物体を渡す, 終了後、carryに |
| place | 物体を設置する, 終了後、carryに |
  
**注意**  
`place`を使う場合は`/current_location`に置く場所のロケーション名をpublishする必要があります。

---
