## デモ用の実行コマンド

Realsense D405での深度画像の取得
```
roslaunch realsense2_camera rs_aligned_depth.launch
```

Realsenseの深度画像から任意距離内のピクセルのみ表示させてpublishするコマンド
```
rosrun imgProc realsense_img_pub
```

ソフトハンドの変形推定
```
roslaunch imgProc deformation.launch
```