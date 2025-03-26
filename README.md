## 実行するときのコマンド

#### 1.Realsense D405での深度画像の取得
RealSense D405をPCに接続した状態で下記コマンドを実行
```
roslaunch realsense2_camera rs_aligned_depth.launch
```

#### 2.入力画像を作成
カメラの場合，透視投影になるので，パースがかかる．
しかし，本手法では，ソフトグリッパのグリッパ表面のみの情報を利用したい．


Realsenseの深度画像から任意距離内のピクセルのみ表示させてpublishするコマンド
```
rosrun imgProc realsense_img_pub
```

ソフトハンドの変形推定
```
roslaunch imgProc deformation.launch
```