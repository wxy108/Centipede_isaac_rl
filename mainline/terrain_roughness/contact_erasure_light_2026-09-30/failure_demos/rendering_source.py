"""Representative saved native Isaac failure trajectories; graphics only."""
from pathlib import Path
import os, ast, json, hashlib, re, time, gc
for key in ['OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS']:
    os.environ[key] = '1'
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path('X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness')
MAIN = ROOT.parent
BASE = ROOT / 'contact_erasure_light_2026-09-30'
OUT = BASE / 'failure_demos'
SIM = BASE / 'simulation_v2'
FPS = 20
DT = .0005

def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n', encoding='utf-8')

parent = MAIN / 'render_demo.py'
source = parent.read_text(encoding='utf-8-sig')
exec(source[:source.index('ap=argparse.ArgumentParser()')], globals())
for node in ast.parse(source).body:
    if isinstance(node, ast.FunctionDef) and node.name in ['poly_for', 'rotation']:
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(parent), 'exec'), globals())

CASES = [
    dict(id='01_wave3_p050_hold10', title='wave 3 · 10 ms · p=0.5 · seed 0',
         attempt='isolated_resume01_s03', job='bin010_p050_b00', source_id=3,
         note='此前重点核对的失败例：跗节高速运动，随后离地继续发散'),
    dict(id='02_wave2_p060_hold20', title='wave 2 · 20 ms · p=0.6 · seed 0',
         attempt='isolated_resume01_s01', job='bin020_p060_b00', source_id=1,
         note='较晚停止的例子：先运行数秒，末段速度快速增长'),
    dict(id='03_wave2_p060_hold50', title='wave 2 · 50 ms · p=0.6 · seed 0',
         attempt='isolated_resume01_s01', job='bin050_p060_b00', source_id=1,
         note='同 wave / p / seed 的另一保持时间；mask 不同，不是单因素因果证明'),
    dict(id='04_wave2p5_p050_hold20', title='wave 2.5 · 20 ms · p=0.5 · seed 0',
         attempt='isolated_resume01_s02', job='bin020_p050_b00', source_id=2,
         note='停止前最后一帧最快关节为 body_pitch，作为另一种观察例子'),
]

if OUT.exists():
    previous = read(OUT / 'VIDEO_REPORT.json')
    assert previous['complete'] and previous['new_physics_steps'] == 0
    assert {v['case']['id'] for v in previous['videos']} == {v['id'] for v in CASES}
    dump(OUT / 'VIDEO_REPORT_preview_v1.json', previous)
OUT.mkdir(exist_ok=True)
(OUT / 'rendering_source.py').write_bytes(Path(__file__).read_bytes())
meta = read(MAIN / 'usd_visual_geometry.json')
geo = np.load(MAIN / 'usd_visual_geometry.npz')
assert sha(meta['asset']) == meta['asset_sha256']
datasets = []
pins = {str(parent): sha(parent), str(MAIN / 'usd_visual_geometry.json'): sha(MAIN / 'usd_visual_geometry.json'),
        str(MAIN / 'usd_visual_geometry.npz'): sha(MAIN / 'usd_visual_geometry.npz')}
for case in CASES:
    folder = SIM / 'runs' / case['attempt'] / case['job']
    active = read(folder / 'active.json')
    assert active['status'] == 'failed'
    env = active['source_ids'].index(case['source_id'])
    failure = read(folder / 'BOUNDARY_FAILURE.json')
    terminal = next(e for e in failure['environments'] if e['source_id'] == case['source_id'])
    with np.load(folder / 'trajectory_partial.npz') as z:
        times = z['t'].copy()
        poses = z['pose'][:, env].copy()
        body_speed = np.linalg.norm(z['body_velocity'][:, env, :, :3].astype(float), axis=-1).max(-1)
        joint_speed = abs(z['v'][:, env]).max(-1).astype(float)
        joint_names = z['joint_names'].tolist()
        fastest = np.argmax(abs(z['v'][:, env]), axis=-1)
        current_names = z['body_names'].tolist()
    if datasets:
        assert current_names == names
    else:
        names = current_names
    assert np.isfinite(poses).all() and np.allclose(np.diff(times), .01)
    # A finite but already exploded state is not rendered as physical motion.
    invalid = (body_speed > 1e6) | (~np.isfinite(body_speed)) | (np.linalg.norm(poses[..., :3], axis=-1).max(-1) > 10)
    if invalid.any():
        end = int(np.flatnonzero(invalid)[0]) - 1
    else:
        end = len(times) - 1
    assert times[end] > 2 and end > 0
    with np.load(folder / 'erasure_mask.npz') as z:
        mask = z['erased'][:, env].copy()
        assert float(z['dt_s']) == DT
    cells = read(folder / 'amplitude_modulation.json')['cells']
    cell = cells[env]
    assert read(folder / 'ERASURE_FIXED_PLANT.json')['passed']
    assert read(folder / 'ERASURE_RESULT.json')['deleted_contact_violations'] == 0
    for filename in ['trajectory_partial.npz', 'erasure_mask.npz', 'BOUNDARY_FAILURE.json',
                     'active.json', 'amplitude_modulation.json', 'ERASURE_FIXED_PLANT.json', 'ERASURE_RESULT.json']:
        pins[str(folder / filename)] = sha(folder / filename)
    datasets.append(dict(case=case, folder=folder, env=env, poses=poses, times=times,
                         body_speed=body_speed, joint_speed=joint_speed, fastest=fastest,
                         joint_names=joint_names, mask=mask, end=end, failure=failure,
                         terminal=terminal, cell=cell, skipped_invalid_times=times[end+1:].tolist()))

indices = {n: i for i, n in enumerate(names)}
spine = [i for i, n in enumerate(names) if n.startswith(('link_body_', 'link_pitch_body_'))]
meshes = []
radii = np.zeros(len(names))
for part in meta['parts']:
    poly, points = poly_for(part)
    j = indices[part['body_name']]
    radii[j] = max(radii[j], float(np.linalg.norm(points, axis=-1).max()))
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(poly)
    mapper.ScalarVisibilityOff()
    foot = re.fullmatch(r'foot_(\d+)_([01])', part['body_name'])
    leg = re.fullmatch(r'link_leg_(\d+)_([LR])_[0-3]', part['body_name'])
    channel = int(foot[1])*2 + int(foot[2]) if foot else int(leg[1])*2 + (leg[2] == 'R') if leg else None
    color = np.asarray(part['diffuse'])
    color = np.where(color <= .0031308, 12.92*color, 1.055*np.maximum(color, 0)**(1/2.4)-.055)
    meshes.append((mapper, j, channel, color, part.get('opacity', 1)))

EYES = [np.array([.14, -.06, .11]), np.array([.20, 0., 0.])]
halfheight = .05
for eye in EYES:
    forward = -eye / np.linalg.norm(eye)
    right = np.cross(forward, [0, 0, 1]); right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    for d in datasets:
        p = d['poses'][:d['end']+1, :, :3].astype(float)
        centers = p[:, spine].mean(1); centers[:, 2] = .008
        rel = p - centers[:, None]
        halfheight = max(halfheight, float((abs(rel @ up) + radii).max())*1.13,
                         float((abs(rel @ right) + radii).max())/(720/540)*1.13)

window = vtk.vtkRenderWindow()
window.SetOffScreenRendering(1); window.SetSize(1440, 540); window.SetMultiSamples(4)
renderers, matrices, actors = [], [], []
for view in range(2):
    ren = vtk.vtkRenderer(); ren.SetViewport(view*.5, 0, (view+1)*.5, 1)
    ren.SetBackground(.92, .945, .96); window.AddRenderer(ren); renderers.append(ren)
    mats = [vtk.vtkMatrix4x4() for _ in names]; matrices.append(mats); aa = []
    for mapper, j, channel, color, opacity in meshes:
        actor = vtk.vtkActor(); actor.SetMapper(mapper); actor.SetUserMatrix(mats[j])
        pr = actor.GetProperty(); pr.SetColor(*color); pr.SetOpacity(opacity)
        pr.SetAmbient(.28); pr.SetDiffuse(.72); pr.SetSpecular(.25); pr.SetSpecularPower(20)
        ren.AddActor(actor); aa.append((actor, channel, color, opacity))
    actors.append(aa)
    plane = vtk.vtkPlaneSource(); plane.SetOrigin(-.8, -.8, 0); plane.SetPoint1(.8, -.8, 0); plane.SetPoint2(-.8, .8, 0); plane.Update()
    mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(plane.GetOutputPort())
    actor = vtk.vtkActor(); actor.SetMapper(mapper); actor.GetProperty().SetColor(.70, .69, .65); ren.AddActor(actor)
    if view == 1:
        slab = vtk.vtkCubeSource(); slab.SetXLength(1.6); slab.SetYLength(1.6)
        slab.SetZLength(.002); slab.SetCenter(0, 0, -.001); slab.Update()
        mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(slab.GetOutputPort())
        actor = vtk.vtkActor(); actor.SetMapper(mapper); actor.GetProperty().SetColor(.60, .62, .63)
        actor.GetProperty().LightingOff(); ren.AddActor(actor)
    lines = vtk.vtkAppendPolyData()
    for coordinate in np.arange(-.8, .801, .01):
        for p1, p2 in [((coordinate, -.8, .00001), (coordinate, .8, .00001)), ((-.8, coordinate, .00001), (.8, coordinate, .00001))]:
            line = vtk.vtkLineSource(); line.SetPoint1(*p1); line.SetPoint2(*p2); line.Update(); lines.AddInputData(line.GetOutput())
    lines.Update(); mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(lines.GetOutputPort())
    actor = vtk.vtkActor(); actor.SetMapper(mapper); actor.GetProperty().SetColor(.58, .61, .62); actor.GetProperty().LightingOff(); ren.AddActor(actor)
    cam = ren.GetActiveCamera(); cam.ParallelProjectionOn(); cam.SetParallelScale(halfheight)
    cam.SetViewUp(0, 0, 1); cam.SetClippingRange(.001, 3)
    light = vtk.vtkLight(); light.SetLightTypeToCameraLight(); light.SetIntensity(.9); ren.AddLight(light)

capture = vtk.vtkWindowToImageFilter(); capture.SetInput(window); capture.SetInputBufferTypeToRGB(); capture.ReadFrontBufferOff()
font = ImageFont.truetype('C:/Windows/Fonts/msyh.ttc', 24)
small = ImageFont.truetype('C:/Windows/Fonts/msyh.ttc', 20)
titlefont = ImageFont.truetype('C:/Windows/Fonts/msyhbd.ttc', 30)
receipt = dict(complete=False, pid=os.getpid(), started_unix=time.time(), new_physics_steps=0,
               backend='VTK OpenGL rasterization of original USD visuals at saved native Isaac poses',
               fps=FPS, rendering_source_sha256=sha(__file__), sources=pins, videos=[], camera_halfheight_m=halfheight,
               selection='Four illustrative failures covering10/20/50ms, early/late stop and tarsal/body fastest-joint observations. Not a random representative statistical sample.',
               fixed_parameters='Existing cluster A/B, full Sep17 wave4 PD, CPU PhysX TGS64/16, physics0.5ms, CPG1ms unchanged.',
               limits='Poses sampled10ms; no interpolation.0.1x replay repeats each stored frame twice at20fps. Invalid terminal pose not rendered as physical motion; its observed values are shown separately. No6s displacement is supplied for failed trials.')
dump(OUT / 'VIDEO_REPORT.json', receipt)

def frame_for(d, index, phase):
    pose = d['poses'][index]
    t = float(d['times'][index])
    mask = d['mask'][max(0, round(t/DT)-1)]
    center = pose[spine, :3].mean(0).astype(float); center[2] = .008
    for view, ren in enumerate(renderers):
        for j, matrix in enumerate(matrices[view]):
            rot = rotation(pose[j, 3:].astype(float))
            for a in range(3):
                for b in range(3): matrix.SetElement(a, b, float(rot[a, b]))
                matrix.SetElement(a, 3, float(pose[j, a]))
            matrix.SetElement(3, 3, 1)
        for actor, channel, color, opacity in actors[view]:
            erased = channel is not None and mask[channel]
            prop = actor.GetProperty(); prop.SetColor(*([.98, .28, .51] if erased else color)); prop.SetOpacity(.48 if erased else opacity)
        cam = ren.GetActiveCamera(); cam.SetPosition(*(center+EYES[view])); cam.SetFocalPoint(*center)
    window.Render(); capture.Modified(); capture.Update()
    pixels = vtk_to_numpy(capture.GetOutput().GetPointData().GetScalars()).reshape(540, 1440, 3)[::-1].copy()
    assert pixels[:, :720].std() > 5 and pixels[:, 720:].std() > 5
    canvas = Image.new('RGB', (1440, 900), '#f5f7fa'); draw = ImageDraw.Draw(canvas)
    draw.text((22, 10), d['case']['title'] + f' | t={t:.3f} s', font=titlefont, fill='#173449')
    draw.text((22, 52), phase + ' · 原 Isaac 保存姿态回放 · 粉色半透明腿：碰撞关闭', font=font, fill='#39576a')
    draw.text((22, 90), '斜视 · 10 mm 地面参考网格', font=small, fill='#39576a'); draw.text((745, 90), '水平侧视 · 地面 z=0', font=small, fill='#39576a')
    canvas.paste(Image.fromarray(pixels), (0, 126))
    draw.text((22, 684), f'刚体最大线速率 {d["body_speed"][index]:.3g} m/s      关节最大速率 {d["joint_speed"][index]:.3g} rad/s', font=font, fill='#173449')
    draw.text((22, 725), f'此帧最快关节：{d["joint_names"][int(d["fastest"][index])]}      擦除 {int(mask.sum())}/38 条腿', font=small, fill='#39576a')
    draw.text((22, 764), d['case']['note'], font=small, fill='#526a7b')
    draw.text((22, 802), '固定原 PD / amplitude / 0.5 ms物理 / 1 ms CPG；姿态每10 ms保存，未插值，身体碰撞保留。', font=small, fill='#526a7b')
    progress = max(0, min(1, t/d['failure']['t']))
    draw.rounded_rectangle((22, 856, 1418, 872), radius=6, fill='#dce5eb')
    draw.rounded_rectangle((22, 856, 22+1396*progress, 872), radius=6, fill='#6d9fba')
    return canvas

try:
    for d in datasets:
        cache = {}
        def get(index, phase):
            key = (index, phase)
            if key not in cache: cache[key] = frame_for(d, index, phase)
            return cache[key]
        path = OUT / (d['case']['id'] + '.mp4')
        writer = imageio.get_writer(path, fps=FPS, codec='libx264', quality=8, macro_block_size=1,
                                   ffmpeg_params=['-threads', '2', '-movflags', '+faststart', '-preset', 'fast'])
        frames = 0
        normal = list(range(0, d['end']+1, 5))
        start = int(np.searchsorted(d['times'], max(2., float(d['times'][d['end']])-.4)))
        slow = list(range(start, d['end']+1))
        try:
            for index in normal:
                writer.append_data(np.asarray(get(index, '常速 1×（20 fps采样）'))); frames += 1
            intro = get(start, '末尾约0.4秒 · 慢放0.1×')
            for _ in range(10): writer.append_data(np.asarray(intro)); frames += 1
            for index in slow:
                picture = get(index, '末尾约0.4秒 · 慢放0.1×')
                for _ in range(2): writer.append_data(np.asarray(picture)); frames += 1
            poster = get(d['end'], '最后可显示保存帧 · 定格（不是新增仿真时间）').copy()
            draw = ImageDraw.Draw(poster)
            draw.rectangle((0, 0, 1440, 126), fill='#fff0ee')
            draw.text((22, 10), f'数值发散停止 | t={d["failure"]["t"]:.4f} s | ' + d['case']['title'], font=titlefont, fill='#ad342e')
            draw.text((22, 56), f'终止状态：body {d["terminal"]["max_body_linear_speed_m_s"]:.3g} m/s；joint {d["terminal"]["max_abs_joint_velocity_rad_s"]:.3g} rad/s', font=font, fill='#ad342e')
            draw.text((22, 94), f'画面定格于 t={d["times"][d["end"]]:.3f}s；终止失真姿态未用于动画，不补造中间运动。', font=small, fill='#8c514c')
            poster.save(OUT / (d['case']['id'] + '_poster.jpg'), quality=92)
            for _ in range(40): writer.append_data(np.asarray(poster)); frames += 1
        finally:
            writer.close()
        count = 0
        with imageio.get_reader(path) as reader:
            video_meta = reader.get_meta_data()
            for count, pixels in enumerate(reader, 1):
                assert pixels.shape == (900, 1440, 3) and pixels.std() > 5
        assert count == frames and abs(video_meta['duration'] - frames/FPS) < .03
        receipt['videos'].append(dict(path=str(path), sha256=sha(path), case=d['case'], source=str(d['folder']),
                                      source_environment_index=d['env'], source_ids=read(d['folder']/'active.json')['source_ids'],
                                      wave=d['cell']['wave'], p=d['cell']['p'], hold_ms=round(d['cell']['erase_bin_s']*1000),
                                      mask_seed=d['cell']['mask_seed'], failure_t_s=d['failure']['t'],
                                      last_rendered_pose_t_s=float(d['times'][d['end']]), skipped_invalid_pose_t_s=d['skipped_invalid_times'],
                                      normal_saved_frame_indices=normal, slow_saved_frame_indices=slow, frames=count,
                                      duration_s=frames/FPS, full_decode_passed=True, manual_review_pending=True))
        dump(OUT / 'VIDEO_REPORT.json', receipt)
        print('Rendered saved failure', d['case']['id'], 'frames', frames, flush=True)
        cache.clear(); gc.collect()
finally:
    window.Finalize()
receipt.update(complete=True, wall_s=time.time()-receipt['started_unix'])
dump(OUT / 'VIDEO_REPORT.json', receipt)
