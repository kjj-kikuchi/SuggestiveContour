
# Suggestive Contour


<img src = "images/bunny_ls3loop.png" width = 32%><img src = "images/bunny_ls3loop_c.png" width = 32%><img src = "images/bunny_ls3loop_sc.png" width = 33%>

非写実的レンダリング (NPR: Non-Photorealistic Rendering) における代表的な特徴線の計算手法である Suggestive Contours for Conveying Shape [DeCarlo et al 2003] の実装．図は左から入力メッシュ，輪郭線，輪郭線とsuggestive contour．

NPRにおける特徴線の代表的な手法を学び，NPRに対する理解を深めることを目的として実装した．

* 使用言語：C++
* 使用ライブラリ：Eigen（線形代数ライブラリ）

### 輪郭線
NPRとは，人間が描く絵画やイラストのような特徴を持つ画像を生成する技術である．3次元形状の輪郭線は，NPRにおけるスタイル付けの基礎をなす重要な曲線である．

3次元形状に対する輪郭線は，視線ベクトルと法線が直交する点の集合である．


### Suggestive Contour
輪郭線はNPRにおいて重要な要素であるが，輪郭線のみで形状の際立った特徴を表現することは難しい．suggestive contourは特徴線の一つで，主に輪郭線と組み合わせて用いられる．視点位置によって線が変化するという特徴があり，これにより自然な結果を生み出すことができる．

suggestive contourは次のように定義される．
```math
\kappa_r = 0 \ \mathrm{and} \ \nabla_{\bm{w}}\kappa_r>0
```
ここで$\nabla_{\bm{w}}\kappa_r$は，$\kappa_r$のベクトル$\bm{w}$に沿った方向微分である．

三角形メッシュに対しては線形補間を用いて線を描画する．まず，各頂点に対して$\kappa_r$の値を計算する．これは微分幾何学におけるオイラーの定理により求められる．
$$
\kappa_r(\bm{p}) = \kappa_1(\bm{p}) \cos^2\phi + \kappa_2(\bm{p}) \sin^2\phi
$$
ここで$\kappa_1, \kappa_2$はそれぞれ最大，最小の主曲率であり，$\phi$は$\bm{w}(\bm{p})$と$\kappa_1$に対応する主方向との角度である．次に三角形内で$\kappa_r$を線形補間し，その零点を見つける．その後，得られた線分の集合に対して$\nabla_{\bm{w}}\kappa_r>0$の判定をする．これは，三角形内で$\kappa_r$の勾配ベクトル計算し，頂点で平均化することで計算できる．

### 実装
次の手順でsuggestive contourを実装した．
* 入力：三角形メッシュ$\mathcal{M}$ (OBJファイル)，視点位置
* 出力：suggestive contourとなる頂点・辺集合 (OBJファイル)  

1. 視点位置，角度に関する閾値，微分値に関する閾値を入力する．
1. 入力メッシュの最小包含球が単位球になるよう正規化する．
1. 各頂点に対して主曲率を計算し，微分幾何学におけるオイラーの定理を用いて$\kappa_r$を求める．
1. 各頂点の勾配ベクトルを計算し，方向微分$\nabla_{\bm{w}}\kappa_r$を求める．
1. 各面内で$\kappa_r$を線形補間し，$\kappa_r=0$となる点を見つける．その点が$\nabla_{\bm{w}}\kappa_r>0$であれば，suggestive contourの頂点・辺リストに追加する．


### 実行例

```
Enter a viewpoint: 10 0 0               // 視点位置
Enter angle threshold : 0.22            // 角度の閾値，π/2なら0.5と入力
Enter derivative threshold : 0.02       // 微分値の閾値
```
