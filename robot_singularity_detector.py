import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, TextBox

# 로봇 링크 길이 설정
L1, L2 = 1.0, 0.8

# 초기 관절각 (라디안)
theta1_init, theta2_init = np.pi/4, np.pi/4

# 자코비안 계산 함수
def compute_jacobian(theta1, theta2):
    # 위치 벡터 p = [x; y]
    # x = L1*cos(theta1) + L2*cos(theta1+theta2)
    # y = L1*sin(theta1) + L2*sin(theta1+theta2)
    # J = [∂x/∂θ1  ∂x/∂θ2;  ∂y/∂θ1  ∂y/∂θ2]
    J11 = -L1*np.sin(theta1) - L2*np.sin(theta1+theta2)
    J12 = -L2*np.sin(theta1+theta2)
    J21 =  L1*np.cos(theta1) + L2*np.cos(theta1+theta2)
    J22 =  L2*np.cos(theta1+theta2)
    return np.array([[J11, J12],
                     [J21, J22]])

# 순방향 운동학으로 말단 위치 계산
def forward_kinematics(theta1, theta2):
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)
    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)
    return (0,0), (x1,y1), (x2,y2)

# 그래프 설정
fig, ax = plt.subplots()
plt.subplots_adjust(left=0.25, bottom=0.35)
(line,) = ax.plot([], [], '-o', lw=2)
warning_text = ax.text(0.5, 1.02, '', transform=ax.transAxes,
                       color='red', fontsize=14, ha='center')

ax.set_xlim(- (L1+L2) * 1.2, (L1+L2) * 1.2)
ax.set_ylim(- (L1+L2) * 1.2, (L1+L2) * 1.2)
ax.set_aspect('equal')
ax.set_title('2-DOF 로봇 시뮬레이션 (Singularity 감지)')

# 슬라이더 축
ax_theta1 = plt.axes([0.25, 0.25, 0.65, 0.03])
ax_theta2 = plt.axes([0.25, 0.20, 0.65, 0.03])

slider_theta1 = Slider(ax_theta1, 'θ1', 0, 2*np.pi, valinit=theta1_init)
slider_theta2 = Slider(ax_theta2, 'θ2', 0, 2*np.pi, valinit=theta2_init)

# 업데이트 함수
def update(val):
    θ1 = slider_theta1.val
    θ2 = slider_theta2.val

    # 포워드 기구학
    p0, p1, p2 = forward_kinematics(θ1, θ2)

    # 그래프 갱신
    xs = [p0[0], p1[0], p2[0]]
    ys = [p0[1], p1[1], p2[1]]
    line.set_data(xs, ys)

    # 자코비안 및 행렬식 계산
    J = compute_jacobian(θ1, θ2)
    detJ = np.linalg.det(J)

    # 특이점 여부 판단: 임계값(threshold) 설정
    threshold = 1e-3
    if abs(detJ) < threshold:
        warning_text.set_text(f'SINGULARITY! det(J)={detJ:.2e}')
    else:
        warning_text.set_text(f'det(J)={detJ:.2e}')

    fig.canvas.draw_idle()

# 슬라이더 이벤트 연결
slider_theta1.on_changed(update)
slider_theta2.on_changed(update)

# 리셋 버튼
ax_reset = plt.axes([0.8, 0.025, 0.1, 0.04])
btn_reset = Button(ax_reset, 'Reset', hovercolor='0.975')
def reset(event):
    slider_theta1.reset()
    slider_theta2.reset()
btn_reset.on_clicked(reset)

# 초기 업데이트 및 표시
update(None)
plt.show()
