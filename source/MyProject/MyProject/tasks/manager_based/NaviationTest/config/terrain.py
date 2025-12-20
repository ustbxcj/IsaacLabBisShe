"""Configuration for box terrains."""

import isaaclab.terrains as terrain_gen
from isaaclab.terrains import TerrainGeneratorCfg
#This file is design to generate terrain
# 创建一个只包含 Box 地形的配置
BOX_TERRAINS_CFG = TerrainGeneratorCfg(
    size=(8.0, 8.0),           # 地形尺寸
    border_width=20.0,         # 边界宽度
    num_rows=10,               # 行数
    num_cols=20,               # 列数
    horizontal_scale=0.1,      # 水平缩放
    vertical_scale=0.005,      # 垂直缩放
    slope_threshold=0.75,      # 坡度阈值
    use_cache=False,           # 不使用缓存
    # curriculum_cfg=TerrainGeneratorCfg.TerrainCurriculumCfg(
    #     difficulty_scales=[0.0, 1.0],  # 难度范围
    # ),
    sub_terrains={
        # 只使用 Box 地形
        "boxes": terrain_gen.MeshBoxTerrainCfg(
            proportion=1.0,  # 100% 概率生成 Box 地形
            box_height_range=(0.1, 0.5),  # 箱子高度范围
            platform_width=2.0,  # 中心平台宽度
            double_box=True,  # 双层箱子
            size=(8.0, 8.0),  # 子地形尺寸
        ),
    },
)
"""Box terrains configuration."""

# 简单坑洞（训练初期）
EASY_PIT_TERRAINS_CFG = TerrainGeneratorCfg(
    size=(8.0, 8.0),
    border_width=15.0,
    num_rows=8,
    num_cols=16,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    sub_terrains={
        "shallow_pit": terrain_gen.MeshPitTerrainCfg(
            proportion=1.0,
            pit_depth_range=(0.1, 0.4),  # 浅坑
            platform_width=2.0,          # 大平台
            double_pit=False,
        ),
    },
)

# 中等坑洞
MEDIUM_PIT_TERRAINS_CFG = TerrainGeneratorCfg(
    size=(10.0, 10.0),
    border_width=20.0,
    num_rows=10,
    num_cols=20,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    sub_terrains={
        "medium_pit": terrain_gen.MeshPitTerrainCfg(
            proportion=1.0,
            pit_depth_range=(0.3, 0.8),  # 中等深度
            platform_width=1.5,
            double_pit=False,
        ),
    },
)

# 困难坑洞（带双层）
HARD_PIT_TERRAINS_CFG = TerrainGeneratorCfg(
    size=(12.0, 12.0),
    border_width=25.0,
    num_rows=12,
    num_cols=24,
    horizontal_scale=0.1,
    vertical_scale=0.006,  # 稍大的垂直缩放
    sub_terrains={
        "hard_pit": terrain_gen.MeshPitTerrainCfg(
            proportion=1.0,
            pit_depth_range=(0.5, 1.2),  # 深坑
            platform_width=1.0,          # 小平台
            double_pit=True,            # 双层
        ),
    },
)
#复杂地形
ROUGH_TERRAINS_CFG = TerrainGeneratorCfg(
    size=(8.0, 8.0),
    border_width=20.0,
    num_rows=10,
    num_cols=20,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    sub_terrains={
        "pyramid_stairs": terrain_gen.MeshPyramidStairsTerrainCfg(
            proportion=0.2,
            step_height_range=(0.05, 0.23),
            step_width=0.3,
            platform_width=3.0,
            border_width=1.0,
            holes=False,
        ),
        "pyramid_stairs_inv": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.2,
            step_height_range=(0.05, 0.23),
            step_width=0.3,
            platform_width=3.0,
            border_width=1.0,
            holes=False,
        ),
        "boxes": terrain_gen.MeshRandomGridTerrainCfg(
            proportion=0.2, grid_width=0.45, grid_height_range=(0.05, 0.2), platform_width=2.0
        ),
        "random_rough": terrain_gen.HfRandomUniformTerrainCfg(
            proportion=0.2, noise_range=(0.02, 0.10), noise_step=0.02, border_width=0.25
        ),
        "hf_pyramid_slope": terrain_gen.HfPyramidSlopedTerrainCfg(
            proportion=0.1, slope_range=(0.0, 0.4), platform_width=2.0, border_width=0.25
        ),
        "hf_pyramid_slope_inv": terrain_gen.HfInvertedPyramidSlopedTerrainCfg(
            proportion=0.1, slope_range=(0.0, 0.4), platform_width=2.0, border_width=0.25
        ),
    },
)
"""Rough terrains configuration."""


# #Ai生成的特殊地形
# NAVIGATION_PIT_TERRAINS_CFG = TerrainGeneratorCfg(
#     size=(15.0, 15.0),  # 更大的地形，适合长距离导航
#     border_width=30.0,
#     num_rows=15,
#     num_cols=30,
#     horizontal_scale=0.1,
#     vertical_scale=0.005,
#     # curriculum_cfg=TerrainGeneratorCfg.TerrainCurriculumCfg(
#     #     difficulty_scales=[0.0, 1.0],
#     #     difficulty_proportions=[
#     #         [0.0, 0.2],   # 非常浅的坑
#     #         [0.2, 0.5],   # 浅坑
#     #         [0.5, 0.8],   # 中等坑
#     #         [0.8, 1.0],   # 深坑
#     #     ],
#     # ),
#     #后续应该开启课程学习
#     sub_terrains={
#         "navigation_pit": terrain_gen.MeshPitTerrainCfg(
#             proportion=1.0,
#             pit_depth_range=(0.2, 1.0),
#             platform_width=1.0,  # 小平台，增加难度
#             double_pit=False,
#             # 添加平坦区域用于起始和目标位置
#             flat_patch_sampling={
#                 "start": terrain_gen.MeshPitTerrainCfg.FlatPatchSamplingCfg(
#                     num_patches=1,
#                     patch_width_range=(0.5, 1.0),
#                     location="border",  # 起始位置在边界
#                 ),
#                 "goal": terrain_gen.MeshPitTerrainCfg.FlatPatchSamplingCfg(
#                     num_patches=1,
#                     patch_width_range=(0.5, 1.0),
#                     location="center",  # 目标位置在中心
#                 )
#             }
#         ),
#     },
# )