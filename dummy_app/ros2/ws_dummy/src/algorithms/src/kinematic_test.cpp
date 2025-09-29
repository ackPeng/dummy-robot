#include <iostream>
#include <cmath>
#include <cfloat>
#include "algorithms/6dof_kinematic.hpp"

static bool isOrthonormal(const float R[9], float tol=1e-4f) {
    for (int i=0;i<3;++i) {
        float n = std::sqrt(R[i]*R[i] + R[3+i]*R[3+i] + R[6+i]*R[6+i]);
        if (std::fabs(n - 1.f) > tol) return false;
    }
    float cx = R[1]*R[8] - R[2]*R[7];
    float cy = R[2]*R[6] - R[0]*R[8];
    float cz = R[0]*R[7] - R[1]*R[6];
    if (std::fabs(cx - R[3]) > tol ||
        std::fabs(cy - R[4]) > tol ||
        std::fabs(cz - R[5]) > tol) return false;
    return true;
}

static float degDiff(float a, float b) {
    float d = fmodf(a - b, 360.0f);
    if (d > 180.f) d -= 360.f;
    if (d < -180.f) d += 360.f;
    return std::fabs(d);
}

static bool jointMatch(const DOF6Kinematic::Joint6D_t& a,
                       const DOF6Kinematic::Joint6D_t& b,
                       float tolDeg=2.0f) {
    for (int i=0;i<6;++i)
        if (degDiff(a.a[i], b.a[i]) > tolDeg) return false;
    return true;
}

static void printPose(const DOF6Kinematic::Pose6D_t& p) {
    std::cout << "Pos(m): (" << p.X << ", " << p.Y << ", " << p.Z << ")\n";
    std::cout << "Euler(rad): A=" << p.A << " B=" << p.B << " C=" << p.C << "\n";
    if (p.hasR) {
        std::cout << "R = [";
        for (int i=0;i<9;++i) {
            std::cout << p.R[i];
            if (i!=8) std::cout << ", ";
        }
        std::cout << "]\n";
    }
}

int main() {
    // DH: 与注释中一致
    DOF6Kinematic kin(0.109f, 0.035f, 0.146f, 0.115f, 0.052f, 0.072f);

    int fail = 0;

    // 1. 零位 FK
    {
        DOF6Kinematic::Joint6D_t q(0,0,0,0,0,0);
        DOF6Kinematic::Pose6D_t pose;
        if (!kin.SolveFK(q, pose)) {
            std::cerr << "[FAIL] 零位 SolveFK 返回 false\n";
            return 1;
        }
        printPose(pose);
        if (std::isnan(pose.X) || std::isnan(pose.Y) || std::isnan(pose.Z)) {
            std::cerr << "[FAIL] 零位位姿出现 NaN\n";
            fail++;
        }
        if (!isOrthonormal(pose.R)) {
            std::cerr << "[FAIL] 零位旋转矩阵非正交\n";
            fail++;
        }
    }

    // 2. 单一样本 FK→IK 回代
    {
        DOF6Kinematic::Joint6D_t q(0.f, -73.f, 180.f, 0.f, 0.f, 0.f);
        DOF6Kinematic::Pose6D_t pose;
        if (!kin.SolveFK(q, pose)) {
            std::cerr << "[FAIL] 样本 FK 失败\n";
            fail++;
        } else {
            std::cout << "\nSample FK:\n";
            printPose(pose);
        }
        DOF6Kinematic::IKSolves_t sols{};
        if (!kin.SolveIK(pose, q, sols)) {
            std::cerr << "[FAIL] 样本 IK 失败\n";
            fail++;
        } else {
            bool ok = false;
            for (int i=0;i<8 && !ok;i++) {
                if (jointMatch(q, sols.config[i], 2.0f)) ok = true;
            }
            if (!ok) {
                std::cerr << "[FAIL] IK 未找到接近原关节的解\n";
                fail++;
            } else {
                std::cout << "[OK] 回代找到匹配 IK 解\n";
            }
        }
    }

    // 3. 腕部奇异（第5轴=0）
    {
        DOF6Kinematic::Joint6D_t q(30.f, -40.f, 70.f, 10.f, 0.f, 120.f);
        DOF6Kinematic::Pose6D_t pose;
        if (!kin.SolveFK(q, pose)) {
            std::cerr << "[FAIL] 奇异位形 FK 失败\n";
            fail++;
        }
        DOF6Kinematic::IKSolves_t sols{};
        if (!kin.SolveIK(pose, q, sols)) {
            std::cerr << "[FAIL] 奇异位形 IK 失败\n";
            fail++;
        } else {
            bool ok = false;
            for (int i=0;i<8 && !ok;i++) {
                if (jointMatch(q, sols.config[i], 3.0f)) ok = true;
            }
            if (!ok) {
                std::cerr << "[FAIL] 奇异位形未匹配回原关节\n";
                fail++;
            } else {
                std::cout << "[OK] 奇异位形回代成功\n";
            }
        }
    }

    if (fail == 0) {
        std::cout << "\nAll basic checks passed.\n";
        return 0;
    } else {
        std::cerr << "\nFailures: " << fail << "\n";
        return 2;
    }
}