# NHKロボコン2026 
# R2
# 経路生成　A*
# KFSシミュレーション

# 乱数設定
import random

# KFSの種類を細分化
EMPTY = 0      # 0: 空（回収後）
R2_KFS = 1     # 1: R2 KFS（回収ターゲット）
OBSTACLE_R1 = 2  # 2: 障害物 (R1 KFS)
OBSTACLE_FAKE = 3 # 3: 障害物 (Fake KFS)
UNKNOWN = -1   # -1: ロボットがまだスキャンしていない未知のエリア

# ブロックの設定
BLOCK_COORDS = {
    1: (0, 0), 2: (0, 1), 3: (0, 2),
    4: (1, 0), 5: (1, 1), 6: (1, 2),
    7: (2, 0), 8: (2, 1), 9: (2, 2),
    10: (3, 0), 11: (3, 1), 12: (3, 2),
}

入口のブロック= {1, 2, 3}
出口のブロック = {10, 11, 12}
境界のブロック = {1, 2, 3, 4, 6, 7, 9, 10, 11, 12}
COORDS_TO_BLOCK = {v: k for k, v in BLOCK_COORDS.items()}
ENTRANCE_BLOCKS_COORDS = {BLOCK_COORDS[i] for i in 入口のブロック}
EXIT_BLOCKS_COORDS = {BLOCK_COORDS[i] for i in 出口のブロック}



#　初期化
class KFS:
    def __init__(self,  kfs_sort = None, position = None):
        self.kfs_sort = kfs_sort
        self.position = position
        self.r1_choice = []
        self.r2_choice = []
        self.choice = []

    def __eq__(self, other):
        return self.position == other.position



# --- シミュレーション: KFS配置 ---
def kfs_place():
    force_trap=False
    print("--- [シミュレーション] 相手チームがKFSを配置しています... ---")
    if force_trap:
        print("--- [シミュレーション] 妨害戦略 : 相手チームが1,2,3 に R2 KFS を配置しませんでした ---")


    # KFSを置くところ
    kfs_map = [[EMPTY for _ in range(3)] for _ in range(4)]
    available_blocks = list(BLOCK_COORDS.keys())
    
    r1を置ける場所 = [i for i in r1を置ける場所 if(i in 境界のブロック)]
    # R1 KFS (3こ)
    if force_trap:       
        r1_choice = []
        r1_choice.extend(入口のブロック) # 罠ブロック ＝ 入口ブロック
        r1を置ける場所 = [i for i in r1を置ける場所 if (i not in 入口のブロック)] # RESET
        print()
        r1_choice.extend(random.sample(r1を置ける場所, 3 - len(r1_choice))) 
    else:
        r1を置ける場所 = [i for i in r1を置ける場所 if (i in 境界のブロック)]
        r1_choice = random.sample(r1を置ける場所, 3)
    for i in r1_choice:
        kfs_map[BLOCK_COORDS[i][0]][BLOCK_COORDS[i][1]] = OBSTACLE_R1
        available_blocks.remove(i)
        continue

    # Fake KFS (1こ)
    フェイクを置く場所 = [i for i in available_blocks if i not in 入口のブロック]
    fake_choice = random.choice(フェイクを置く場所, 1)
    for i in fake_choice:
        kfs_map[BLOCK_COORDS[i][0]][BLOCK_COORDS[i][1]] = OBSTACLE_FAKE
        available_blocks.remove(i)
        continue

    # R2 KFS (4こ)
    r2を置く場所 = available_blocks
    if force_trap:
        r2を置く場所 = [i for i in available_blocks if i not in 入口のブロック]
    r2_choices = random.sample(r2を置く場所, 4)
    for i in r2_choices:
        kfs_map[BLOCK_COORDS[i][0]][BLOCK_COORDS[i][1]] = R2_KFS
        available_blocks.remove(i)
        continue
    
    print("--- [シミュレーション] 配置完了 ---")
    return kfs_map

if __name__ == "__main__":
    kfs_place()