import numpy as np
import cv2
import pickle
from pathlib import Path


def to_h(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    H = np.eye(4)
    H[:3, :3] = R
    H[:3, 3] = t.reshape(3)
    return H


def build_motions(poses_used, R_t2c_list, t_t2c_list):
    n = min(len(poses_used), len(R_t2c_list), len(t_t2c_list))
    if n < 2:
        raise ValueError("Need >= 2 samples")

    Tg = [np.array(P) for P in poses_used]  # base->tcp
    T_t2c = [to_h(np.array(R), np.array(t))
             for R, t in zip(R_t2c_list, t_t2c_list)]  # target->cam
    T_c2t = [np.linalg.inv(T) for T in T_t2c]  # cam->target

    A1 = [np.linalg.inv(Tg[i]) @ Tg[i+1] for i in range(n-1)]
    A2 = [np.linalg.inv(Tg[i+1]) @ Tg[i] for i in range(n-1)]

    B1 = [np.linalg.inv(T_t2c[i]) @ T_t2c[i+1] for i in range(n-1)]
    B2 = [np.linalg.inv(T_t2c[i+1]) @ T_t2c[i] for i in range(n-1)]
    B3 = [np.linalg.inv(T_c2t[i]) @ T_c2t[i+1]
          for i in range(n-1)]  # equals T_t2c[i] @ inv(T_t2c[i+1])
    B4 = [np.linalg.inv(T_c2t[i+1]) @ T_c2t[i] for i in range(n-1)]

    def split_RT(Ts):
        Rs = [T[:3, :3] for T in Ts]
        ts = [T[:3, 3] for T in Ts]
        return Rs, ts

    variants = []
    for a_name, A in [("A1", A1), ("A2", A2)]:
        for b_name, B in [("B1", B1), ("B2", B2), ("B3", B3), ("B4", B4)]:
            Rg, tg = split_RT(A)
            Rt, tt = split_RT(B)
            variants.append(((a_name, b_name), (Rg, tg, Rt, tt)))
    return variants


def main():
    data_path = Path('src/hand_eye_calibration_data.pkl')
    if not data_path.exists():
        print(f"Missing {data_path}")
        return

    with open(data_path, 'rb') as f:
        calib = pickle.load(f)

    poses_used = calib.get('poses_used', [])
    R_t2c_list = calib.get('R_target2cam', [])
    t_t2c_list = calib.get('t_target2cam', [])

    variants = build_motions(poses_used, R_t2c_list, t_t2c_list)

    results = []
    for (a_name, b_name), (Rg, tg, Rt, tt) in variants:
        try:
            R, t = cv2.calibrateHandEye(
                Rg, tg, Rt, tt, method=cv2.CALIB_HAND_EYE_TSAI)
            H = np.eye(4)
            H[:3, :3] = R
            H[:3, 3] = t.reshape(3)
            trans_norm = float(np.linalg.norm(H[:3, 3]))
            # test point 25 cm ahead in camera
            p_cam = np.array([0.0, 0.0, 0.25, 1.0])
            p_tcp = H @ p_cam
            dist_tcp = float(np.linalg.norm(p_tcp[:3]))
            results.append({
                'A': a_name, 'B': b_name,
                'trans_norm': trans_norm,
                'p_tcp': p_tcp[:3].tolist(),
                'dist_from_origin_tcp': dist_tcp,
                'H': H,
            })
        except Exception as e:
            results.append({'A': a_name, 'B': b_name, 'error': str(e)})

    # Rank by closeness of translation norm to 0.2 m
    ranked = [r for r in results if 'H' in r]
    ranked.sort(key=lambda r: abs(r['trans_norm'] - 0.2))

    print("Variant sweep results (top 8):")
    for r in ranked[:8]:
        print(
            f"A={r['A']} B={r['B']} | ||t||={r['trans_norm']:.3f} | p_tcp={np.array(r['p_tcp'])}")

    if ranked:
        best = ranked[0]
        out_path = Path('src/hand_eye_matrix_final.npy')
        np.save(out_path, best['H'])
        print('Selected best variant:', best['A'], best['B'])
        print('Saved to', out_path)
        print('p_tcp for (0,0,0.25):', np.array(best['p_tcp']))
    else:
        print('No valid variants produced a solution')


if __name__ == '__main__':
    main()
