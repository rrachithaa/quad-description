import math

L1 = 0.20  # thigh length (m)
L2 = 0.20  # shin length (m)


def leg_fk(thigh, knee, l1=L1, l2=L2):
    """Foot position (x, z) relative to the hip, from joint angles."""
    phi = thigh + knee
    return (-l1 * math.sin(thigh) - l2 * math.sin(phi),
            -l1 * math.cos(thigh) - l2 * math.cos(phi))


def leg_ik(x, z, l1=L1, l2=L2):
    """Joint angles (thigh, knee) that put the foot at (x, z) relative to the hip."""
    r = math.hypot(x, z)
    r = min(max(r, 0.05), (l1 + l2) * 0.999)
    alpha = math.atan2(-x, -z)
    cos_beta = (l1 * l1 + r * r - l2 * l2) / (2 * l1 * r)
    cos_gamma = (l1 * l1 + l2 * l2 - r * r) / (2 * l1 * l2)
    beta = math.acos(max(-1.0, min(1.0, cos_beta)))
    gamma = math.acos(max(-1.0, min(1.0, cos_gamma)))
    return alpha + beta, -(math.pi - gamma)


if __name__ == '__main__':
    worst = 0.0
    for x, z in [(0.0, -0.306), (0.05, -0.30), (-0.05, -0.28), (0.0, -0.22), (0.08, -0.25)]:
        th, kn = leg_ik(x, z)
        fx, fz = leg_fk(th, kn)
        err = math.hypot(fx - x, fz - z)
        worst = max(worst, err)
        print(f'target ({x:+.3f}, {z:+.3f}) -> thigh {th:+.3f}, knee {kn:+.3f} -> back to ({fx:+.3f}, {fz:+.3f})')
    print(f'worst round-trip error: {worst:.2e} m')
