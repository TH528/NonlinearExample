# NonlinearExample

## 概要
- CLFによる非線形制御のシミュレーションプログラム
- 剛体姿勢の（almost）大域的漸近安定化問題を数値計算（オイラー法）により解く．
- シミュレータのコア部分は[ControlSimCore](https://github.com/TH528/ControlSimCore)をベースにしている

## シミュレータ概要
- C++による制御システムシミュレータ
- シミュレーション結果はCSVデータ

## 言語
- C++17

## 環境
Windows 10 or macOS
1. Visual Studio 2019 (Communityで動作確認済み)
1. CMake
1. ターミナル

## CMakeによる実行方法
```
mkdir build
cd build
cmake ..
make clean all
./main
```

## ターミナルからの実行方法
```
g++ -Wall -std=c++17 main.cpp -o main
./main
```

## 構成
- main.cpp: 制御システムや制御則を記述
- solver/: 微分方程式を数値計算するライブラリ
- csv/: CSVファイルの書き込みを行うライブラリ