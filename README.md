# FingerVision後処理フィルタ用RTコンポーネント

大阪電気通信大学  
澤﨑 悠太，升谷 保博

## 概要
- [FingerVision](http://akihikoy.net/p/fv.html)用の[ソフトウェア](https://github.com/akihikoy/fingervision)に含まれる[後処理フィルタ用のROSのノード](https://github.com/akihikoy/fingervision/blob/master/fingervision/scripts/fv_filter1.py)をRTミドルウェアのコンポーネントとして移植したものです．
- Pythonのスクリプトに含まれている関数の多くはオリジナルコードをそのまま利用しています．
- 以下で動作を確認
  - Windows 10 64bit
  - OpenRTM-aist 1.2.0 64bit
  - Python 3.7.3
  - numpy 1.17.0
- Ubuntu 16.04でも実行できることを簡単に確認しています．

## インストール（Windowsの場合）

- Pythonをインストール．
- OpenRTM-aistをインストール．
- コマンドプロンプトで`pip install numpy`を実行．
- `idlcompile.bat`を実行．

## 仕様

独自のデータ型を[idl/FingerVision.idl](idl/FingerVision.idl)で定義しています．独自データ型の各メンバの名前と意味はオリジナルのトピックとほぼ同じです．`stamp`だけは，RTミドルウェアの慣習に従って`tm`に置き換えています．

### 入力ポート

- blob_moves （データ型： FingerVision::BlobMoves）
- prox_vision （データ型： FingerVision::ProxVision）

### 出力ポート

- fv_filter1_wrench （データ型： FingerVision::Filter1Wrench）
- fv_filter1_objinfo （データ型： FingerVision::Filter1ObjInfo）

## 使い方（Windowsの場合）

- Naming Serviceを起動しておきます．
- トップディレクトリの[FingerVisionFilter.bat](FingerVisionFilter.bat)を実行．

以上．
