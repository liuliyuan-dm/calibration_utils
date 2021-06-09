### HAND EYE CALIBRATION

### Step 1: Build an AX = XB Problem

![image-20210609174140249](hand%20eye%20calibration.assets/image-20210609174140249.png)

假设k时刻，机械臂相对于世界坐标系的位姿为 $$ A_k$$，相机相对于世界坐标系的位姿为$$B_k$$，相机相对于机械臂的位姿为 X($T_{AB}$)，那么，对于k时刻有
$$
A_k X = B_k  \tag{1}
$$
同样地，对k+1时刻有
$$
A_{k+1}X = B_{k+1} \tag{2}
$$
进一步整理得到
$$
A_{k+1}^{-1}A_kX=XB_{k+1}^{-1}B_k \tag{3}
$$
令$A=A_{k+1}^{-1}A_k$,$B=B_{k+1}^{-1}B_k$, 上式即为：
$$
AX=XB \tag{4}
$$
其中，A和B表示相邻时刻从k到k+1的机械臂和相机的局部运动。机械臂的局部运动可以通过其运动模型推算得到，相机的局部运动可以通过外部的标定板辅助得到。由此，手眼标定问题转化为了$AX=XB$求解问题，我们称k时刻与k+1时刻构成的$AX=XB$方程为一个测量。



### Step 2: Constraint analysis (at least two sets of measurement needed)

分别考虑旋转矩阵R求解和平移向量t求解所需要的方程数量。

假设：
$$
A=\left[ \begin{array}{cc}
   R_A & t_A \\
   0 & 1
  \end{array} \right],
  B=\left[\begin{array}{cc}
R_B & t_B \\
0 & 1
 \end{array} \right],
 X=\left[\begin{array}{cc}
R_X & t_X \\
0&1
\end{array}\right]\tag{5}
$$
则$AX=XB$转化为
$$
\begin{cases}
R_AR_X=R_XR_B\\
R_At_X+t_A=R_Xt_B+t_X\\
\end{cases}\tag{6}
$$

***旋转矩阵求解与平移部分无关，但平移部分求解与旋转有关。***

进一步，由上述等式条件可以得到：
$$
R_A=R_XR_BR_X^T\tag{7}
$$
***旋转矩阵特性：***

1. 旋转矩阵有且只有一个实数特征值1；

2. 旋转矩阵是行列式为1的正交矩阵（$RR^T=I$）；

3. 不考虑单位矩阵，旋转矩阵的实数特征值对应的特征向量即是axis-angle表达式的旋转轴，旋转矩阵的迹与旋转角度相关：$\theta=arccos(\frac{tr(R)-1}{2})$

    

***至少需要几组测量？***

“Robot Sensor Calibration: Solving AX = XB on the Euclidean Group”中证明至少需要两组测量。且要求两组测量不能平行。



### Step 3: Solve the rotation matrix X by least squares method

由上述推导可以知道，手眼标定本质是求解方程$R_AR_X=R_XR_B$，$R_X\in SO(3)$，则可等价于$R_A=R_XR_BR_X^T$，对仅旋转矩阵构成的方程两边取对数得到：
$$
logR_A = logR_XR_BR_X^T\tag{8}
$$
令$logR_A = [\alpha]$，$logR_B = [\beta]$，则上式转化为：
$$
[\alpha]=R_X[\beta]R_X^T=[R_X\beta]\tag{9}
$$
进而得到：
$$
\alpha = R_X \beta\tag{10}
$$
存在多组观测时，可以将公式(11)转化为最小二乘问题：
$$
min\sum\limits_{i=1}^{k}||R_X \beta_i - \alpha_i||^2 \tag{11}
$$
***大于两组测量时，线性最小二乘问题存在理论最优解，为：***
$$
M=\sum\limits_{i=1}^{k}\beta_i\alpha_i^T \tag{12}
$$
为了保证$X$满足旋转矩阵特性（正交，行列式为1），可令X为：
$$
R_X = (MM^T)^{-\frac{1}{2}}M \tag{13}
$$
***$\alpha$，$\beta$ 分别是手眼姿态旋转矩阵的Rodrigues矢量（  $  [\theta n_x,\theta n_y,\theta n_z]^T$，twist）形式表示。***

公式（14）可通过SVD分解的形式求解如下：
$$
\begin{split}&M^TM = Q\Lambda Q^{-1} \\\Longrightarrow \ 
&(M^TM)^{-1/2}=Q\Lambda^{-1/2}Q^{-1},\ \ \ 
&\Lambda^{-1/2}=diag(\frac{1}{\sqrt{\lambda _1}},\frac{1}{\sqrt{\lambda _2}},\frac{1}{\sqrt{\lambda _3}})\end{split} \tag{14}
$$
***仅有两组测量时，其解为：***
$$
\begin{split}&\Alpha = \{\alpha_1,\alpha_2, \alpha_1 \times \alpha_2 \}\\
&\Beta =\{\beta_1,\beta_2, \beta_1 \times \beta_2 \}\\
&R_X = \Alpha\Beta^{-1}
\end{split} \tag{15}
$$

### Step 4: Solve the translation vector $t_X$

由公式(6)得到，求解外参平移量$ t_X$等价于求解：
$$
R_{A_i}t_X+t_{A_i}=R_Xt_{B_i}+t_{X} \tag{16}
$$
移项得到：
$$
(I-R_{A_i})t_X = t_{A_i}-R_Xt_{B_i} \tag{17}
$$
将多组方程一并记做：
$$
Ct_X=d \tag{18}
$$
其中，
$$
C = \left[
\begin{matrix} 
I-R_{A_1}\\
I-R_{A_2}\\
\vdots\\
I-R_{A_P}\\
\end{matrix}
\right], \ \ \ 

d = \left[
\begin{matrix} 
t_{A_1}-R_{X}t_{B_1}\\
t_{A_2}-R_{X}t_{B_2}\\
\vdots\\
t_{A_P}-R_{X}t_{B_P}\\
\end{matrix}
\right]\tag{19}
$$
***又因为$C$不一定可逆，故在公式（17）两端同时乘以$C^T$，即：***
$$
C^TC\ t_X = C^Td \tag{20}
$$
由此得到外参平移量$t_X$的最小二乘解为：
$$
t_X = (C^TC)^{-1}C^Td\tag{21}
$$



### Reference

[1] F. Park, B. Martin, "Robot Sensor Calibration: Solving AX = XB on the Euclidean Group." [[CALI] 1994 Robot sensor calibration: solving AX=XB on the Euclidean group.pdf ](reference/[CALI] 1994 Robot sensor calibration: solving AX=XB on the Euclidean group.pdf) (the method above)

***Other methods:***

[2] R. Y. Tsai and R. K. Lenz, "A new technique for fully autonomous and efficient 3D robotics hand/eye calibration." 

[3] R. Horaud, F. Dornaika, "Hand-Eye Calibration."

[4] N. Andreff, R. Horaud, B. Espiau, "On-line Hand-Eye Calibration."

[5] K. Daniilidis, "Hand-Eye Calibration Using Dual Quaternions."