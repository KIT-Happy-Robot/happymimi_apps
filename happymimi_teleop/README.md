# happymimi_teleop
## Overview
happymimi_teleopは、happymimiの足回り制御に関するROSパッケージです。<br>
このパッケージが提供するものは以下の３つです。
- 足回り制御処理をまとめたPythonモジュール
- キーボードコントローラ制御ノード
- ジョイスティックコントローラ制御ノード

## Description
### base_controlモジュール
base_controlは、足回り制御処理をまとめたPythonモジュールです。<br>
BaseControlクラスにある、以下２つのメソッドを適時呼び出してください。

- translateDist(dist, speed)
> - 前進、後進処理を実行します。第一引数は必ず入力してください。<br>
> - 第二引数は必須ではないですが、並進速度が指定可能です（デフォルトは0.2[m/s]）。

|  第一引数  |  第二引数  |
| :-- | :-- |
| 進行距離[m] |  並進速度[m/s]  |

- rotateAngle(deg, precision, speed, time_out)
> - 左右回転処理を実行します。第一引数は必ず入力してください。<br>
> - 回転方向を変える際は回転角度にマイナスをつけてください。<br>
> - 第二、第三、第四引数は必須ではないですが、それぞれ指定できます。<br>
> - 第二引数は、精度を指定することができます。1を入力すると少数第1位まで精度が上がります。
> - 第三引数は、角速度を指定することができます。
> - 第四引数は、制限時間を指定することができます。10と指定したら10秒で処理が終わります。

|  第一引数  |  第二引数  | 第三引数 | 第四引数 |
| :-- | :-- | :-- | :-- |
| 回転角度[deg] |  精度  |  角速度[rad/s]  |  制限時間[s]  |

|　第二引数　|　誤差　|
| :-- | :-- |
|　0　|　&plusmn; 0.9  |
|　1　|　&plusmn; 0.09  |
|　2　|　&plusmn; 0.009 |

### キーボードコントローラ制御ノード
キーボードから足回りを操作できます。


### ジョイスティックコントローラ制御ノード
ジョイスティックコントローラから制御ができます。<br>
ジョイスティックは[DUALSHOCK 4](https://www.playstation.com/ja-jp/accessories/dualshock-4-wireless-controller/)のキー配置に対応しています。

## Usage
### キーボードコントローラの起動
キーボードを使う場合<br>
`roslaunch happymimi_bringup minimal.launch`<br>
`roslaunch happymimi_teleop keyboard_teleop.launch`


コントローラのキー配置
```
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
```


### ジョイスティックコントローラの起動
ジョイスティックコントローラを使うときは、コントローラをUSBに接続した後に下記コマンドを実行してください。<br>
`roslaunch happymimi_bringup minimal.launch`<br>
`roslaunch happymimi_teleop joycon_teleop.launch`

