from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


def configure_chinese_font():
    candidates = [
        "SimHei",
        "Microsoft YaHei",
        "Noto Sans CJK SC",
        "Noto Sans CJK JP",
        "WenQuanYi Zen Hei",
        "PingFang SC",
        "Source Han Sans SC",
        "Arial Unicode MS",
    ]
    available = {font.name for font in font_manager.fontManager.ttflist}
    for name in candidates:
        if name in available:
            plt.rcParams["font.sans-serif"] = [name]
            break
    else:
        plt.rcParams["font.sans-serif"] = ["DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False


def add_box(ax, xy, width, height, title, subtitle="", facecolor="#F7F7F7"):
    box = FancyBboxPatch(
        xy,
        width,
        height,
        boxstyle="round,pad=0.02,rounding_size=0.02",
        linewidth=1.1,
        edgecolor="black",
        facecolor=facecolor,
    )
    ax.add_patch(box)
    x = xy[0] + width / 2
    y = xy[1] + height / 2
    text = title if not subtitle else f"{title}\n{subtitle}"
    ax.text(x, y, text, ha="center", va="center", fontsize=11)
    return box


def add_arrow(ax, start, end):
    ax.add_patch(
        FancyArrowPatch(
            start,
            end,
            arrowstyle="->",
            mutation_scale=12,
            linewidth=1.2,
            color="black",
        )
    )


def plot_framework(output_path):
    configure_chinese_font()
    fig, ax = plt.subplots(figsize=(13, 8.5), dpi=300)
    fig.patch.set_facecolor("white")
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    ax.set_title("RH-Proposed-v2 方法总体框架", fontsize=17, pad=16)

    add_box(ax, (0.04, 0.78), 0.18, 0.14, "输入层", "栅格地图\n机器人当前位置\n巡检任务集合", "#EAF2F8")
    add_box(ax, (0.29, 0.78), 0.16, 0.14, "路径代价层", "A* 路径代价计算", "#FDEDEC")
    add_box(
        ax,
        (0.52, 0.76),
        0.21,
        0.18,
        "基础任务评分层",
        "基础任务效用函数\npriority、risk、abnormal_weight\n distance_cost、complexity_cost、energy_cost",
        "#E8F8F5",
    )
    add_box(ax, (0.79, 0.78), 0.17, 0.14, "候选集层", "混合候选池生成", "#FEF9E7")

    add_box(
        ax,
        (0.17, 0.48),
        0.22,
        0.16,
        "滚动时域规划层",
        "Beam Search / 滚动时域搜索\nhorizon、beam_width、candidate_pool_size",
        "#F4ECF7",
    )
    add_box(
        ax,
        (0.46, 0.44),
        0.30,
        0.24,
        "序列评估层",
        "序列级目标函数评估\n累计收益\n累计路径惩罚\n累计时间惩罚\n优先级加权完成时间惩罚\n异常加权完成时间惩罚\n前K步高优先级奖励",
        "#E8F6F3",
    )
    add_box(ax, (0.81, 0.50), 0.15, 0.12, "输出执行层", "输出当前最优任务\n机器人执行并更新当前位置", "#FDEDEC")
    add_box(ax, (0.58, 0.14), 0.24, 0.14, "反馈层", "异常触发\nabnormal_weight 更新\n下一轮滚动规划", "#FEF5E7")

    add_arrow(ax, (0.22, 0.85), (0.29, 0.85))
    add_arrow(ax, (0.45, 0.85), (0.52, 0.85))
    add_arrow(ax, (0.73, 0.85), (0.79, 0.85))
    add_arrow(ax, (0.875, 0.78), (0.875, 0.62))
    add_arrow(ax, (0.79, 0.56), (0.76, 0.56))
    add_arrow(ax, (0.46, 0.56), (0.39, 0.56))
    add_arrow(ax, (0.28, 0.78), (0.28, 0.64))
    add_arrow(ax, (0.58, 0.44), (0.70, 0.28))
    add_arrow(ax, (0.70, 0.21), (0.88, 0.50))
    add_arrow(ax, (0.58, 0.21), (0.30, 0.48))
    add_arrow(ax, (0.30, 0.48), (0.60, 0.44))

    ax.text(0.87, 0.33, "闭环反馈", fontsize=11)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=300, facecolor="white", bbox_inches="tight")
    plt.close(fig)

    print(f"输出图片路径: {output_path}")
    print("是否成功保存: 是")
    print("使用的绘图方式: matplotlib")


def main():
    base_dir = Path(__file__).resolve().parent
    output_path = base_dir / "results" / "figures" / "fig8_framework.png"
    plot_framework(output_path)


if __name__ == "__main__":
    main()
