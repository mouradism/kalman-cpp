#pragma once

#include "radar/Matrix.hpp"
#include "radar/TrackerTypes.hpp"
#include <cmath>

namespace radar::tracker {

class ExtendedKalmanFilter {
public:
    static void predict(
        const math::Vector<4>& x,
        const math::Matrix<4, 4>& P,
        float dt,
        float sigma_a,
        math::Vector<4>& out_x_pred,
        math::Matrix<4, 4>& out_P_pred
    ) {
        math::Matrix<4, 4> F = math::Matrix<4, 4>::identity();
        F(0, 2) = dt;
        F(1, 3) = dt;

        const float dt2 = dt * dt;
        const float dt3 = dt2 * dt;
        const float dt4 = dt3 * dt;
        const float sa2 = sigma_a * sigma_a;

        math::Matrix<4, 4> Q;
        const float q_pos = sa2 * (dt4 / 4.0f);
        const float q_pos_vel = sa2 * (dt3 / 2.0f);
        const float q_vel = sa2 * dt2;

        Q(0, 0) = q_pos;     Q(0, 2) = q_pos_vel;
        Q(1, 1) = q_pos;     Q(1, 3) = q_pos_vel;
        Q(2, 0) = q_pos_vel; Q(2, 2) = q_vel;
        Q(3, 1) = q_pos_vel; Q(3, 3) = q_vel;

        out_x_pred = F * x;
        out_P_pred = F * P * F.transpose() + Q;
        out_P_pred.makeSymmetric();
    }

    static void computeObservation(
        const math::Vector<4>& x,
        math::Vector<3>& out_z_pred
    ) {
        const float px = x(0, 0);
        const float py = x(1, 0);
        const float vx = x(2, 0);
        const float vy = x(3, 0);

        float rho = std::sqrt(px * px + py * py);
        constexpr float EPSILON = 0.1f; // Singularity guard
        if (rho < EPSILON) {
            rho = EPSILON;
        }

        const float theta = std::atan2(py, px);
        const float vr = (px * vx + py * vy) / rho;

        out_z_pred(0, 0) = rho;
        out_z_pred(1, 0) = theta;
        out_z_pred(2, 0) = vr;
    }

    static void computeJacobian(
        const math::Vector<4>& x,
        math::Matrix<3, 4>& out_H
    ) {
        const float px = x(0, 0);
        const float py = x(1, 0);
        const float vx = x(2, 0);
        const float vy = x(3, 0);

        float rho = std::sqrt(px * px + py * py);
        constexpr float EPSILON = 0.1f; // Singularity guard
        if (rho < EPSILON) {
            rho = EPSILON;
        }

        const float rho2 = rho * rho;
        const float rho3 = rho2 * rho;

        // Row 0: d(rho)/dx
        out_H(0, 0) = px / rho;
        out_H(0, 1) = py / rho;
        out_H(0, 2) = 0.0f;
        out_H(0, 3) = 0.0f;

        // Row 1: d(theta)/dx
        out_H(1, 0) = -py / rho2;
        out_H(1, 1) = px / rho2;
        out_H(1, 2) = 0.0f;
        out_H(1, 3) = 0.0f;

        // Row 2: d(vr)/dx
        out_H(2, 0) = (py * (vx * py - vy * px)) / rho3;
        out_H(2, 1) = (px * (vy * px - vx * py)) / rho3;
        out_H(2, 2) = px / rho;
        out_H(2, 3) = py / rho;
    }

    static void computeInnovation(
        const math::Vector<3>& z_meas,
        const math::Vector<3>& z_pred,
        math::Vector<3>& out_y
    ) {
        out_y(0, 0) = z_meas(0, 0) - z_pred(0, 0);
        out_y(1, 0) = math::wrapToPi(z_meas(1, 0) - z_pred(1, 0));
        out_y(2, 0) = z_meas(2, 0) - z_pred(2, 0);
    }

    static bool computeInnovationCovariance(
        const math::Matrix<4, 4>& P_pred,
        const math::Matrix<3, 4>& H,
        const math::Matrix<3, 3>& R,
        math::Matrix<3, 3>& out_S,
        math::Matrix<3, 3>& out_S_inv,
        float& out_det_S
    ) {
        out_S = H * P_pred * H.transpose() + R;
        out_S.makeSymmetric();
        return math::invert3x3(out_S, out_S_inv, out_det_S);
    }

    static void computeKalmanGain(
        const math::Matrix<4, 4>& P_pred,
        const math::Matrix<3, 4>& H,
        const math::Matrix<3, 3>& S_inv,
        math::Matrix<4, 3>& out_K
    ) {
        out_K = P_pred * H.transpose() * S_inv;
    }

    static float mahalanobisDistanceSq(
        const math::Vector<3>& y,
        const math::Matrix<3, 3>& S_inv
    ) {
        const math::Vector<3> Sy = S_inv * y;
        return math::dot(y, Sy);
    }

    static void updateJoseph(
        const math::Vector<4>& x_pred,
        const math::Matrix<4, 4>& P_pred,
        const math::Vector<3>& y,
        const math::Matrix<3, 4>& H,
        const math::Matrix<3, 3>& R,
        const math::Matrix<4, 3>& K,
        math::Vector<4>& out_x_up,
        math::Matrix<4, 4>& out_P_up
    ) {
        out_x_up = x_pred + K * y;

        const math::Matrix<4, 4> I = math::Matrix<4, 4>::identity();
        const math::Matrix<4, 4> I_KH = I - (K * H);

        out_P_up = (I_KH * P_pred * I_KH.transpose()) + (K * R * K.transpose());
        out_P_up.makeSymmetric();
    }
};

} // namespace radar::tracker

