# grasping_object
## Overview
Dynamixel_controllerを用いた物体把持関係をまとめたパッケージ  

## Description
このパッケージが提供する機能は主に以下の1つです。
- 認識から把持までの一連のタスクの実行
  
## Usage
### Recognition To Grasping
  
  
| Communication | Name | Type | Request | Result |
| :---: | :---: | :---: | :---: | :---: |
| Service | /recognition_to_grasping | [RecognitionToGrasping](https://github.com/KIT-Happy-Robot/happymimi_manipulation/blob/master/happymimi_manipulation_msgs/srv/RecognitionToGrasping.srv) | string型: `target_name`<br>[happymimi_msgs/StrInt型](https://github.com/KIT-Happy-Robot/happymimi_robot/blob/develop/happymimi_msgs/msg/StrInt.msg): `sort_option` | bool型: `result` |
  
**target_nameの種類**  
| target_name | Contents |
| :---: | :--- |
| 特定の名前(ex. cupとかbottleとか) | 指定した名前の物体を把持する |
| any | 把持可能物体を適当に把持する |
  
**sort_optionの種類（任意）**  
| data | Contents |
| :---: | --- |
| left | 画面左から順に並び替える |
| center | 画面中央から順に並び替える |
| right | 画面右から順に並び替える |
  
| num | Contents |
| :---: | --- |
| 数値 | ソートしたリストを元に、指定した数値番目の物体を把持する |
  
え？わかりづらい？  
例えばcupが複数あって、左から2番目のcupを把持したいときは、  
```
target_name = 'cup'
sort_option.data = 'left'
sort_option.num = 1
```
ってやるといいよ  
    
**Resultについて**  
把持できたらTrue、失敗したらFlase  

---
