# NHKロボコン2026
# R2
# matplot実装

# インポート
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# 自作ファイル　(行数減らしたいのでインポート）
import astar
from astar import height_map, path
import kfs 




# 3Dマップ生成
def plot_3d_maze_path(height_map, path):
    rows, cols = height_map.shape  # マップ生成
    dx = dy = 0.9  # ブロックのサイズ

    fig = plt.figure(figsize=(5, 5))
    ax = Axes3D(fig)
     #ここを追加↓
    fig.add_axes(ax)

    # マップのカラー設定
    for r in range(rows):
        for c in range(cols):
            z = 0
            h = height_map[r, c]
            color = 'yellowgreen'   # 薄緑
            if h == 1.0:
                color = 'darkgreen' # 緑
            if h == 2.0:
                color = 'green'     # 緑      
            if path and (r, c) in path:
                color = 'yellow'    # 黄
            if state == R2_KFS: 
                color = 'blue'      # 青
            elif state == OBSTACLE_R1: 
                color = 'red'       # 赤
            elif state == OBSTACLE_FAKE: 
                color = 'purple'    # 紫
            elif state == UNKNOWN: 
                color = 'grey'      # 灰
            else:
                color = 'lightgrey' # 灰

        #マップの比率設定    
        ax.bar3d(c, r, z, dx, dy, h, color=color, alpha=0.7)
        ax.set_box_aspect((1,1, 0.2))
    # ラベル
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Height')
    ax.set_title("3D Maze with Blocks")
    plt.show()


astar.main()



