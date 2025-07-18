import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

# 로봇 링크 길이 설정
a1, a2 = 1.0, 1.0

# 초기 관절각 (라디안)
theta1_init, theta2_init = 0,0

# 각 링크 색상 설정
link1_color = 'red'    # 첫 번째 링크 색
link2_color = 'blue'   # 두 번째 링크 색
ee_color    = 'green'  # End-Effector 색
marker_style = 'o'

# 자코비안 계산
def jacobian(theta1, theta2):
    j11 = -a1 * np.sin(theta1) - a2 * np.sin(theta1 + theta2)
    j12 = -a2 * np.sin(theta1 + theta2)
    j21 =  a1 * np.cos(theta1) + a2 * np.cos(theta1 + theta2)
    j22 =  a2 * np.cos(theta1 + theta2)
    return np.array([[j11, j12], [j21, j22]])

# 순방향 운동학
def forward_kinematics(theta1, theta2):
    x1 = a1 * np.cos(theta1)
    y1 = a1 * np.sin(theta1)
    x2 = x1 + a2 * np.cos(theta1 + theta2)
    y2 = y1 + a2 * np.sin(theta1 + theta2)
    return (0, 0), (x1, y1), (x2, y2)

def check_singularity(J):
    """특이점 확인"""
    det_J = np.linalg.det(J)
    return abs(det_J) < 1e-6, det_J  # 행렬식이 0에 가까운 경우

# Figure & Axes 설정
fig = plt.figure(figsize=(6, 8))
ax_arm = fig.add_axes([0.1, 0.45, 0.8, 0.5])
ax_det = fig.add_axes([0.1, 0.1, 0.8, 0.25])

# 로봇 팔 플롯 초기화 (개별 링크 및 EE)
line1,     = ax_arm.plot([], [], '-', lw=2, color=link1_color, marker=marker_style)
line2,     = ax_arm.plot([], [], '-', lw=2, color=link2_color, marker=marker_style)
ee_marker, = ax_arm.plot([], [], marker_style, color=ee_color, markersize=6)

ax_arm.set_xlim(-(a1 + a2) * 1.2, (a1 + a2) * 1.2)
ax_arm.set_ylim(-(a1 + a2) * 1.2, (a1 + a2) * 1.2)
ax_arm.set_aspect('equal')
ax_arm.grid(True)

# 슬라이더 축 (위치 조정)
ax_t1 = fig.add_axes([0.3, 0.50, 0.4, 0.03])
ax_t2 = fig.add_axes([0.3, 0.45, 0.4, 0.03])
s_t1   = Slider(ax_t1, 'Theta 1', - np.pi ,  np.pi, valinit=theta1_init)
s_t2   = Slider(ax_t2, 'Theta 2', -  np.pi ,  np.pi, valinit=theta2_init)

# det-plot 초기 설정
ax_det.set_title('Singularity Determinant')
ax_det.set_xlabel('det(J)')
ax_det.set_ylabel('det_Jacobian')
ax_det.axhline(0, color='k', linewidth=1)

# 업데이트 함수
def update(val):
    theta1, theta2 = s_t1.val, s_t2.val

    # 포워드 기구학
    p0, p1, p2 = forward_kinematics(theta1, theta2)

    # 링크별 그래프 갱신
    line1.set_data([p0[0], p1[0]], [p0[1], p1[1]])
    line2.set_data([p1[0], p2[0]], [p1[1], p2[1]])
    # End-Effector 표시: 시퀀스로 전달
    ee_marker.set_data([p2[0]], [p2[1]])

    # 자코비안 및 det 계산
    is_singularity,detJ = check_singularity(jacobian(theta1, theta2))

    # det-plot 갱신
    ax_det.clear()
    ax_det.set_title('Singularity Determinant')
    ax_det.set_xlabel('det(J)')
    ax_det.set_ylabel('det_Jacobian')
    ax_det.set_xlim(-a1 * a2, a1 * a2)
    ax_det.set_ylim(-1.0, 1.0)
    ax_det.axhline(0, color='k', linewidth=1)

    # det 부호에 따른 채우기
    if detJ < 0:
        ax_det.axhspan(detJ, 0, facecolor='green', alpha=0.5)
    else:
        ax_det.axhspan(0, detJ, facecolor='green', alpha=0.5)

    # 특이점 경고
    if is_singularity:
        ax_det.text(0, 0, 'SINGULARITY!', color='red', ha='center', va='bottom', fontsize=14)
    else:
        ax_det.text(0, detJ/2, f'{detJ:.2e}', ha='center', va='center')

    fig.canvas.draw_idle()

# 이벤트 연결
s_t1.on_changed(update)
s_t2.on_changed(update)

# 리셋 버튼
ax_reset   = plt.axes([0.8, 0.025, 0.1, 0.04])
btn_reset = Button(ax_reset, 'Reset', hovercolor='0.975')
def reset(event): s_t1.reset(); s_t2.reset()
btn_reset.on_clicked(reset)

# 초기 실행 및 표시
update(None)
plt.show()
