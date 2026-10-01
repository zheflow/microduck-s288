"""Gate 的 8 层。每层一个模块，导出 run(ctx) -> LayerResult。

层的契约（写新层照抄 l0_mesh.py）：
    LAYER = 0..7                     模块级常量
    def run(ctx) -> LayerResult      ctx 见 gate.py:Ctx

ctx 提供：
    ctx.data     dict，12 个 YAML 的 safe_load 结果，键 = 文件名去后缀
    ctx.parts    有序件号列表 ['L01',...,'H03']
    ctx.stl(pid) -> Path | None      cad/duck_s288/ 里该件导出的 STL（**必须查导出文件，不查内存网格**）
    ctx.placed(name) -> Path | None  cad/duck_s288/placed/ 里的世界坐标件
    ctx.mesh(path)                   带缓存的 trimesh 加载（process=False）
    ctx.only                         --parts 限定的件号集合（None = 全部）
    ctx.mjcf()                       duckstructure.kin.load() 的结果，带缓存

铁律（tools/gate/README.md 第 3 节）：
    1. 查导出的 STL，不查内存网格
    2. 每条 Finding 必须给 evidence_n；给不出就用 res.unknown(...)
    3. 采样网格独立于建模用的角度
    4. 拿不到数 = FAIL(BLOCK)，不许静默 PASS 或跳过
    5. 判据从 tolerances.yaml 取，不在层里写死数字
"""
LAYERS = [0, 1, 2, 3, 4, 5, 6, 7]
