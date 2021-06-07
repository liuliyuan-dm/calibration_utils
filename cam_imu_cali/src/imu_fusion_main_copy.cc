#include "imu.h"
//#include "extended_kalman_filter.h"

int main(int argc, char **argv)
{
  Imu imu;
  Vector3d bu, be, dbe;
  Vector3d wu, wm, we, wx, am;
  Quaterniond qgb = Quaterniond(1.0, 0.0, 0.0, 0.0);
  Vector3d nr(0.1, 0.1, 0.1), nw(0.01, 0.01, 0.01), na(0.01, 0.01, 0.01);
  MatrixXd F(6, 6), Phi(6, 6), Pu(6, 6), Pe(6, 6), Q(6, 6), I(6, 6),
      H(3, 6), K(6, 3);
  Matrix3d S, R;
  I.setIdentity();
  Pe = 0.1 * I;
  R = R.setIdentity() * 0.01;
  F.setZero();
  double dt = 0.01; //sample rate: 100hz
  //应该加一个初始化函数，对上面的各个量初始化

  // step0: read txt
  ifstream finw("../data/imu/wb_measure.txt");
  ifstream fina("../data/imu/ab_measure.txt");
  if (!finw || !fina)
  {
    cerr << "Imu data is not found!" << endl;
    return 1;
  }

  vector<Vector3d> vwm, vam;
  while (!finw.eof() && !fina.eof())
  {
    Vector3d dat_w(0), dat_a(0);
    for (int i = 0; i < 3; i++)
    {
      finw >> dat_w[i];
      fina >> dat_a[i];
    }
    vwm.push_back(dat_w);
    vam.push_back(dat_a);
    //cout << dat_w << endl;
    //cout << dat_a << endl;
  }

  uint64_t N;
  N = min(vwm.size(), vam.size());
  ofstream fout_qe("../data/imu/qgb_estimate.txt");
  ofstream fout_we("../data/imu/wb_estimate.txt");
  ofstream fout_be("../data/imu/bias_estimate.txt");
  ofstream fout_Mat("../data/imu/Mat_estimate.txt");

  for (uint64_t i = 1; i < N; i++)
  {
    //cout << i << ":  " << endl;

    wm = vwm[i];
    am = vam[i];
    am = am / am.norm();

    // step1: bu(k+1|k) = be(k|k)
    bu = be;

    // step2: wu(k+1|k) = wm(k+1)-bu(k+1|k)
    wu = wm - bu;

    /* step3: wx = [wu(k+1|k)+we(k)]/2, qu = [1,1/2*dt*wx,1/2*dt*wy,1/2*dt*wz]*qe
            qe,qu: a rotation from world to body, expressed as qwb_e,qwb_u  
    */
    wx = (wu + we) / 2.0;
    Quaterniond dqw = Quaterniond(1.0, 0.5 * wx[0] * dt, 0.5 * wx[1] * dt,
                                  0.5 * wx[2] * dt);
    dqw.normalize();
    qgb = qgb * dqw;
    qgb.normalize();

    /* step4: F  = [ 0,      wx[2], -wx[1], -1,   0,  0; 
                    -wx(2),  0,      wx(0),  0,  -1,  0;
                     wx(1), -wx(0),  0,      0,   0, -1;
                     zeros(3,6)]
  
            Phi = eye(6,6) + F * dt
    */
    F << 0.0, wx[2], -wx[1], -1.0, 0.0, 0.0, -wx[2], 0.0, wx[0], 0.0, -1.0, 0.0, wx[1], -wx[0],
        0.0, 0.0, 0.0, -1.0;

    Phi = I + F * dt;
    //cout << "Phi" << endl;
    //cout << Phi << endl;

    // step5: Q = [sigma_r,0,0,sigma_w]*dt
    Q.diagonal() << nr.cwiseProduct(nr), nw.cwiseProduct(nw);
    Q = Q * dt;

    // step6: P(k+1|k) = Phi*P(k|k)*Phi'+Q
    Pu = Phi * Pe * Phi.transpose() + Q;
    //cout << "Pu" << endl;
    //cout << Pu << endl;

    // step7: H(k+1) = [[Rbw(k+1|k)*ag x] 0]
    Matrix3d Rbw = qgb.toRotationMatrix().transpose();
    H << 0.0, Rbw(2, 2), -Rbw(1, 2), 0.0, 0.0, 0.0, -Rbw(2, 2), 0.0, Rbw(0, 2), 0.0, 0.0, 0.0,
        Rbw(1, 2), -Rbw(0, 2), 0.0, 0.0, 0.0, 0.0;
    //cout << "H" << endl;
    //cout << H << endl;

    // step8: r(k+1) = am(k+1)-au(k+1|k)
    Vector3d ra, au; //要从读进来的原始数据给am赋值, vector3d默认列向量3x1
    au << -Rbw(0, 2), -Rbw(1, 2), -Rbw(2, 2);
    ra = am - au;

    // step9: S(k+1) = H(k+1)*P(k+1|k)*H(k+1)'+R(k+1)
    S = H * Pu * H.transpose() + R;
    //cout << "S" << endl;
    //cout << S << endl;

    // step10: K(k+1) = P(k+1|k)*H(k+1)'*inv(S(k+1))
    K = Pu * H.transpose() * S.inverse();
    //cout << "K" << endl;
    //cout << K << endl;

    // step11: dxe(k+1) = K(k+1)*ra(k+1)
    VectorXd dxe(6, 1), dv(3, 1);
    Quaterniond dqe;
    dxe = K * ra;
    dbe << dxe[3], dxe[4], dxe[5];
    dv << dxe[0] * 0.5, dxe[1] * 0.5, dxe[2] * 0.5;

    if (dv.norm() < 1.0)
      dqe = Quaterniond(sqrt(1.0 - dv.transpose() * dv), dv[0], dv[1],
                        dv[2]);
    else
    {
      double scale = 1.0 / sqrt(1 + dv.transpose() * dv);
      dqe = Quaterniond(1.0 * scale, dv[0] * scale, dv[1] * scale, dv[2] * scale);
    }
    qgb = qgb * dqe;
    //qgb.normalize();

    // step12: be = bu+dbe
    be = bu + dbe;

    // step13: we = wm-be
    we = wm - be;

    // step14: Pe = (I-K*H)*Pu*(I-K*H)'+K*R*K'
    Pe = (I - K * H) * Pu * ((I - K * H).transpose()) + K * R * K.transpose();

    //cout << "Pe" << endl;
    //cout << Pe << endl;

    MatrixXd Temp1, Temp2, Temp3;
    Temp1 = S.inverse();
    Temp2 = K * H;
    Temp3 = I - K * H;

    //cout << "S.inverse" << endl;
    //cout << Temp1 << endl;
    //cout << "K*H" << endl;
    //cout << Temp2 << endl;
    //cout << "I-K*H" << endl;
    //cout << Temp3 << endl;

    fout_qe << qgb.w() << " " << qgb.x() << " "
            << qgb.y()
            << " " << qgb.z() << endl;
    fout_we << "we: " << we[0] << " " << we[1] << " " << we[2] << " | "
            << "wu: " << wu[0] << "  " << wu[1] << "  " << wu[2] << "  "
            << " | "
            << "wx: " << wx[0] << " " << wx[1] << " " << wx[2] << endl;
    fout_be << be[0] << " " << be[1] << " " << be[2] << endl;
    fout_Mat << "Fc: " << F(0, 0) << " " << F(0, 1) << " " << F(0, 2) << " " << F(0, 3) << " " << F(0, 4) << " " << F(0, 5) << "  |  "
             << "Pu: " << Pu(0, 0) << " " << Pu(0, 1) << " " << Pu(0, 2) << " " << Pu(0, 3) << " " << Pu(0, 4) << " " << Pu(0, 5) << "  |  "
             << "Pe: " << Pe(0, 0) << " " << Pe(0, 1) << " " << Pe(0, 2) << " " << Pe(0, 3) << " " << Pe(0, 4) << " " << Pe(0, 5) << endl;
    fout_Mat << "    " << F(1, 0) << " " << F(1, 1) << " " << F(1, 2) << " " << F(1, 3) << " " << F(1, 4) << " " << F(1, 5) << "  |  "
             << "    " << Pu(1, 0) << " " << Pu(1, 1) << " " << Pu(1, 2) << " " << Pu(1, 3) << " " << Pu(1, 4) << " " << Pu(1, 5) << "  |  "
             << "    " << Pe(1, 0) << " " << Pe(1, 1) << " " << Pe(1, 2) << " " << Pe(1, 3) << " " << Pe(1, 4) << " " << Pe(1, 5) << endl;
    fout_Mat << "    " << F(2, 0) << " " << F(2, 1) << " " << F(2, 2) << " " << F(2, 3) << " " << F(2, 4) << " " << F(2, 5) << "  |  "
             << "    " << Pu(2, 0) << " " << Pu(2, 1) << " " << Pu(2, 2) << " " << Pu(2, 3) << " " << Pu(2, 4) << " " << Pu(2, 5) << "  |  "
             << "    " << Pe(2, 0) << " " << Pe(2, 1) << " " << Pe(2, 2) << " " << Pe(2, 3) << " " << Pe(2, 4) << " " << Pe(2, 5) << endl;
    fout_Mat << "    " << F(3, 0) << " " << F(3, 1) << " " << F(3, 2) << " " << F(3, 3) << " " << F(3, 4) << " " << F(3, 5) << "  |  "
             << "    " << Pu(3, 0) << " " << Pu(3, 1) << " " << Pu(3, 2) << " " << Pu(3, 3) << " " << Pu(3, 4) << " " << Pu(3, 5) << "  |  "
             << "    " << Pe(3, 0) << " " << Pe(3, 1) << " " << Pe(3, 2) << " " << Pe(3, 3) << " " << Pe(3, 4) << " " << Pe(3, 5) << endl;
    fout_Mat << "    " << F(4, 0) << " " << F(4, 1) << " " << F(4, 2) << " " << F(4, 3) << " " << F(4, 4) << " " << F(4, 5) << "  |  "
             << "    " << Pu(4, 0) << " " << Pu(4, 1) << " " << Pu(4, 2) << " " << Pu(4, 3) << " " << Pu(4, 4) << " " << Pu(4, 5) << "  |  "
             << "    " << Pe(4, 0) << " " << Pe(4, 1) << " " << Pe(4, 2) << " " << Pe(4, 3) << " " << Pe(4, 4) << " " << Pe(4, 5) << endl;
    fout_Mat << "    " << F(5, 0) << " " << F(5, 1) << " " << F(5, 2) << " " << F(5, 3) << " " << F(5, 4) << " " << F(5, 5) << "  |  "
             << "    " << Pu(5, 0) << " " << Pu(5, 1) << " " << Pu(5, 2) << " " << Pu(5, 3) << " " << Pu(5, 4) << " " << Pu(5, 5) << "  |  "
             << "    " << Pe(5, 0) << " " << Pe(5, 1) << " " << Pe(5, 2) << " " << Pe(5, 3) << " " << Pe(5, 4) << " " << Pe(5, 5) << endl;
  }
  fout_qe.close();
  fout_we.close();
  fout_be.close();
  fout_Mat.close();
}
