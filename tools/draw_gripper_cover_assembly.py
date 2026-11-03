"""그리퍼 덮개(iec 한 덩어리형 D27.5)를 그리퍼에 끼운 모습과 가운데 단면. 실행: ~/cad_env/bin/python tools/draw_gripper_cover_assembly.py → cad/out/gripper_cover_assembly.png"""
import importlib.util, sys, numpy as np, trimesh, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib import font_manager
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
f="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"; font_manager.fontManager.addfont(f); plt.rcParams["font.family"]=font_manager.FontProperties(fname=f).get_name()
spec=importlib.util.spec_from_file_location("g","/home/cho/plugbot/cad/make_gripper_cover.py"); g=importlib.util.module_from_spec(spec); spec.loader.exec_module(g)
d=27.5
cover,info=g.build_iec(d,onepiece=True); cover=g._simplify(cover)
cover.apply_transform(np.linalg.inv(info["gripper_to_print"]))  # 그리퍼 좌표(mm)
fixed,moving=g.jaws(g.Q_CLAMP)
cam=g._part("wrist_roll_follower_so101_camera_mount.stl",(0,-0.000218,0.00095),(3.14159,0,0))
motor=g._part("sts3215_03a_v1.stl",(0.0077,0.0001,-0.0234),(-1.5708,0,0))
parts=[(m.copy(), c) for m,c in ((fixed,"#c9c9cf"),(moving,"#9a9aa3"),(cam,"#e8e8ea"),(motor,"#2b2b30"))]
for m,_ in parts: m.apply_scale(1000.0)
light=np.array([0.4,-0.6,0.7]); light/=np.linalg.norm(light)
def shade(m,hexc,alpha=1.0):
    base=np.array(matplotlib.colors.to_rgb(hexc)); k=0.35+0.65*np.clip(np.abs(m.face_normals@light),0,1)
    return np.c_[base[None,:]*k[:,None], np.full(len(k),alpha)]
fig=plt.figure(figsize=(16,7.5))
ax=fig.add_subplot(1,2,1,projection="3d")
allm=[(m,c,1.0) for m,c in parts]+[(cover,"#2f6fb0",0.55)]
for m,c,a in allm:
    ax.add_collection3d(Poly3DCollection(m.triangles,facecolors=shade(m,c,a),linewidths=0))
ax.set_xlim(-60,60);ax.set_ylim(-60,60);ax.set_zlim(-170,-50);ax.view_init(12,-35);ax.set_box_aspect((1,1,1));ax.set_axis_off()
ax.set_title("체결 모습(덮개 반투명). 그리퍼 끝이 아래(-z)",fontsize=13)
# 단면: y = cy 평면
bx=fig.add_subplot(1,2,2)
cy=info["axis_xy"][1]
def sec(m,c,fill=True):
    s=m.section(plane_origin=(0,cy,0),plane_normal=(0,1,0))
    if s is None: return
    for e in s.discrete: 
        if fill: bx.fill(e[:,0],e[:,2],color=c,alpha=0.9,lw=0)
        else: bx.plot(e[:,0],e[:,2],color=c,lw=1)
sec(cover,"#2f6fb0"); 
for m,c in parts[:2]: sec(m,c)
for m,c in parts[:2]: sec(m,"k",False)
bx.set_aspect("equal"); bx.set_xlim(-45,45); bx.set_ylim(-165,-40)
bx.annotate("칸막이(쐐기)\n움직이는 턱이 눌러 문다",xy=(info["axis_xy"][0]+1,-95),xytext=(18,-120),arrowprops=dict(arrowstyle="->"),fontsize=11)
bx.annotate("고정 턱",xy=(-15,-80),xytext=(-44,-60),arrowprops=dict(arrowstyle="->"),fontsize=11)
bx.annotate("움직이는 턱",xy=(18,-75),xytext=(22,-55),arrowprops=dict(arrowstyle="->"),fontsize=11)
bx.annotate(f"접점 Ø{d} (소켓에 들어감)",xy=(info["axis_xy"][0],-150),xytext=(5,-162),arrowprops=dict(arrowstyle="->"),fontsize=11)
bx.set_title("가운데 단면(파랑 = 덮개)",fontsize=13); bx.set_xlabel("mm"); bx.grid(alpha=0.2)
fig.tight_layout(); out="/home/cho/plugbot/cad/out/gripper_cover_assembly.png"; fig.savefig(out,dpi=95); print(out)
