## 実行するときのコマンド

#### 1.Realsense D405での深度画像の取得
RealSense D405をPCに接続した状態で下記コマンドを実行
```
roslaunch realsense2_camera rs_aligned_depth.launch
```

#### 2.入力画像を作成
カメラの場合，透視投影になるので，パースがかかる．
しかし，本手法では，ソフトグリッパのグリッパ表面のみの情報を利用したい．
ので，深度画像と組み合わせて，カメラから一定の距離のカラー情報のみを保存するプログラムを作っている．

・Color Image: RealSenseの画像

・Processed Image: カメラから一定距離以内のピクセルのみ表示したもの．それ以外は白色で描写

(なお，ノイズの影響や，テクスチャレスな部分のステレオカメラの対応点問題までは考慮できていない)
```
rosrun imgProc realsense_img_pub
```

#### 3. キャリブレーション
変形推定を行う場合，まずはキャリブレーションとして，ハンドがどれくらい曲がるかを取得しておく必要があります(曲げ角度)
```
rosrun imgProc setup
```

ただ，先端の点がうまく追跡できない場合がある(バグ修正未対応ごめんなさい)．
そのときは，src/procCommon.cppのfindBaseAndTip()関数の値を変更してください．
その値を，config/param.yamlのCalibrated bending angleってところに記載してください．
これが終わったら3のターミナルのみ閉じてください．


#### 4. 変形推定
上記手順2で生成した画像を入力画像として変形推定を行う．
なお，deformation.launchの実行に関連するファイルは， src/deformation.cppおよびsrc/procCommon.cppを参照．

・imgOutputEstColor: 変形推定の結果

・Deformation Estimation: その時の結果を描画．'d'キーで結果描写，'r'で描写クリア．
```
roslaunch imgProc deformation.launch
```

