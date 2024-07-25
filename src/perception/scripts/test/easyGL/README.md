在三维图形和计算机视觉中，将三维点转换到二维图像坐标通常涉及几个步骤，包括应用外参矩阵 \([R|t]\) 和内参矩阵 \(K\)。这个过程确实包括了一个最后的步骤，称为“透视除法”，这是从齐次坐标转换到笛卡尔坐标的关键步骤。下面详细解释这个过程：

### 1. 外参矩阵 \([R|t]\)

外参矩阵 \([R|t]\) 将世界坐标系中的三维点转换到相机坐标系。这个变换涉及旋转和平移，可以表示为：
$$

\begin{bmatrix}
X_c \\
Y_c \\
Z_c \\
1
\end{bmatrix}
=
\begin{bmatrix}
R & t \\
0 & 1
\end{bmatrix}
\begin{bmatrix}
X_w \\
Y_w \\
Z_w \\
1
\end{bmatrix} 
$$
其中 $(X_w, Y_w, Z_w)$ 是世界坐标中的点，$(X_c, Y_c, Z_c)$是相机坐标系中的点。
$$
E = \begin{bmatrix}
R & t \\
0 & 1
\end{bmatrix}
$$
$E$为内参矩阵

$$
E^{-1} = \begin{bmatrix}
    R^{t}&-R^{t}t \\
    0&1
\end{bmatrix}
$$
$E^{-1}$外参矩阵的逆矩阵
### 2.内参矩阵

内参矩阵 $k$ 是一个3x3矩阵，通常具有如下形式：
$$
K = \begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}
$$

$$
K^{-1} = \begin{bmatrix}
1/f_x & 0 & -c_x/f_x \\
0 & 1/f_y & -c_y/f_y \\
0 & 0 & 1
\end{bmatrix}
$$

### 投影与逆投影
简单投影矩阵
$$
\begin{bmatrix}
    u \\
    v \\
    0
\end{bmatrix} = \begin{bmatrix}
    f_x & 0 & c_x \\
    0 & f_y & c_y \\
    0 & 0 & 1
\end{bmatrix} 
\begin{bmatrix}
x_c \\
y_c \\
1   \\
\end{bmatrix}
$$
$$
\begin{bmatrix}
    x_c \\
    y_c \\
    z_c \\
    1
\end{bmatrix} = 
\begin{bmatrix}
    R & t \\
0 & 1
\end{bmatrix}\begin{bmatrix}
    x_w \\
    y_w \\
    z_w \\
    1 \\
\end{bmatrix}
$$

逆投影矩阵