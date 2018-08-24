#ifndef COMMON_H
#define COMMON_H

#include "Eigen/Core"
#include "Eigen/Geometry"


template <typename _T> inline void normalizeQuaternion ( Eigen::Matrix< _T, 4 , 1  >& quat )
{
  _T quat_norm = quat.norm();
  quat /= quat_norm;
}


template <typename _T> 
  static inline void computeOmegaSkew( const Eigen::Matrix< _T, 3, 1> &omega, 
                                       Eigen::Matrix< _T, 4, 4> &skew )
{
  skew <<   _T(0),     -omega(0),  -omega(1),  -omega(2),
            omega(0),   _T(0),      omega(2),  -omega(1),
            omega(1),  -omega(2),   _T(0),      omega(0),
            omega(2),   omega(1),  -omega(0),   _T(0);
}

template <typename _T> 
  inline void quatIntegrationStepRK4( const Eigen::Matrix< _T, 4, 1> &quat, 
                                              const Eigen::Matrix< _T, 3, 1> &omega0, 
                                              const Eigen::Matrix< _T, 3, 1> &omega1, 
                                              const _T &dt, Eigen::Matrix< _T, 4, 1> &quat_res )
{
  Eigen::Matrix< _T, 3, 1> omega01 = _T(0.5)*( omega0 + omega1 );
  Eigen::Matrix< _T, 4, 1> k1, k2, k3, k4, tmp_q;
  Eigen::Matrix< _T, 4, 4> omega_skew;
  
  // First Runge-Kutta coefficient
  computeOmegaSkew( omega0, omega_skew );
  k1 = _T(0.5)*omega_skew*quat;
  // Second Runge-Kutta coefficient
  tmp_q = quat + _T(0.5)*dt*k1;
  computeOmegaSkew( omega01, omega_skew );
  k2 = _T(0.5)*omega_skew*tmp_q;
  // Third Runge-Kutta coefficient (same omega skew as second coeff.)
  tmp_q = quat + _T(0.5)*dt*k2;
  k3 = _T(0.5)*omega_skew*tmp_q;
  // Forth Runge-Kutta coefficient
  tmp_q = quat + dt*k3;
  computeOmegaSkew( omega1, omega_skew );
  k4 = _T(0.5)*omega_skew*tmp_q;
  _T mult1 = _T(1.0)/_T(6.0), mult2 = _T(1.0)/_T(3.0);
  quat_res = quat + dt*(mult1*k1 + mult2*k2 + mult2*k3 + mult1*k4);
  normalizeQuaternion(quat_res);
}

#endif
