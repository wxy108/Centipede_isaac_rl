"""Render preselected spawn12 actual saved Isaac poses; never run physics.

Each terrain keeps five fixed wave tiles, including explicit incomplete tiles.
Videos cover source times0..5.95s (120 frames,20fps); static previews use exact6s.
Source geometry and the established original-USD/VTK functions are pinned.
"""
from pathlib import Path
import argparse
import ast
import gc
import hashlib
import json
import os
import re
import shutil
import time

for key in ('OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[key] = '1'

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path('X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness')
MAIN = ROOT.parent
EXPANSION = ROOT / 'libo_cutoff_height_expansion_2026-10-01'
WAVES = [1.5, 2., 2.5, 3., 3.5]
SPAWN = 12
WIDTH, HEIGHT = 1440, 1760
PINS = {
    MAIN / 'render_demo.py': 'f4d58d09df48ee32a676f18147b9a5ca3ffb41a1f76cff5f24a9f44e73b958ee',
    ROOT / 'render_libo_spawn30.py': '58a6c761931ed658fc8ee621360fb4d04194c22beeaf2bf6f233d85091fdbd1e',
    MAIN / 'usd_visual_geometry.json': 'ecb62c0308bf94b603a6ef5b41e08dcd49d40b9b27019fbe4bcd9beb8356a87c',
    MAIN / 'usd_visual_geometry.npz': '2b4515aed8332f844e283b134c6e99404e3db6277e636ee1ac9f938c85ebc400',
}
font = ImageFont.truetype('C:/Windows/Fonts/msyh.ttc', 21)
small = ImageFont.truetype('C:/Windows/Fonts/msyh.ttc', 17)
titlefont = ImageFont.truetype('C:/Windows/Fonts/msyhbd.ttc', 28)


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n', encoding='utf-8')


def load_graphics():
    """Run imports only, then extract the established geometry functions."""
    global geo, meta
    for path, expected in PINS.items():
        assert sha(path) == expected, f'Visual input changed: {path}'
    parent = MAIN / 'render_demo.py'
    source = parent.read_text(encoding='utf-8-sig')
    exec(source[:source.index('ap=argparse.ArgumentParser()')], globals())
    from vtkmodules.util.numpy_support import numpy_to_vtkIdTypeArray
    globals()['numpy_to_vtkIdTypeArray'] = numpy_to_vtkIdTypeArray
    receipt = []
    for path, names in [(parent, ('poly_for', 'rotation')),
                        (ROOT / 'render_libo_spawn30.py', ('ground_mapper',))]:
        tree = ast.parse(path.read_text(encoding='utf-8-sig'))
        for name in names:
            node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name)
            exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), globals())
            receipt.append(dict(path=str(path), source_sha256=sha(path), function=name,
                                ast_sha256=hashlib.sha256(ast.dump(node, include_attributes=False).encode()).hexdigest()))
    meta = read(MAIN / 'usd_visual_geometry.json')
    geo = np.load(MAIN / 'usd_visual_geometry.npz')
    assert sha(meta['asset']) == meta['asset_sha256']
    return receipt


def level(cutoff, height):
    return f"cutoff{cutoff:g}mm_height_x{height:g}".replace('.', 'p')


def key(cutoff, height, wave):
    return f'c{cutoff:g}_h{height:g}_w{wave:g}_s{SPAWN:02d}'


def load_stage(base):
    sim = base / 'simulation'
    campaign_path = sim / 'campaign.json'
    campaign = read(campaign_path)
    assert read(sim / 'supervisor.json')['complete'], 'Native stage must finish before rendering'
    analysis_path = base / 'results/RESULTS.json'
    analysis = read(analysis_path)
    assert analysis['complete']
    assert campaign['fixed_pd_sha256'] == analysis['fixed_pd_sha256']
    assert campaign['profile_sha256'] == analysis['profile_sha256']
    committed_sources = {}
    for path, digest in analysis['source_hashes'].items():
        normalized = os.path.normcase(str(Path(path).resolve()))
        assert normalized not in committed_sources or committed_sources[normalized] == digest
        committed_sources[normalized] = digest

    def committed_hash(path):
        normalized = os.path.normcase(str(Path(path).resolve()))
        assert normalized in committed_sources, f'Analysis did not commit selected source: {path}'
        digest = sha(path)
        assert digest == committed_sources[normalized], f'Selected source differs from committed analysis: {path}'
        return digest

    campaign_digest = committed_hash(campaign_path)
    supervisor_digest = committed_hash(sim / 'supervisor.json')
    assert list(campaign['waves']) == WAVES and len(campaign['spawns']) == 30
    assert campaign['spawns'][SPAWN]['spawn_index'] == SPAWN
    series_path = base / 'results/forward_timeseries.npz'
    series_digest = sha(series_path)
    assert series_digest == analysis['forward_timeseries_sha256'], 'Forward series differs from committed analysis'
    with np.load(series_path) as z:
        series = {name: z[name].copy() for name in z.files}
    cells = {(float(c['cutoff_mm']), float(c['height_scale']), float(c['wave'])): c
             for c in analysis['cells'] if c['spawn_index'] == SPAWN}
    failures = {(float(c['cutoff_mm']), float(c['height_scale']), float(c['wave'])): c
                for c in analysis['failed_cells'] if c['spawn_index'] == SPAWN}
    assert not set(cells) & set(failures)
    loaded = {}
    names = None
    datasets = []
    pins = {str(campaign_path): campaign_digest, str(sim / 'supervisor.json'): supervisor_digest,
            str(analysis_path): sha(analysis_path), str(series_path): series_digest}
    source_receipts = []
    for terrain in sorted(campaign['terrains'], key=lambda r: (r['cutoff_mm'], r['height_scale'])):
        cutoff, height = float(terrain['cutoff_mm']), float(terrain['height_scale'])
        terrain_file = Path(terrain['path'])
        assert sha(terrain_file) == terrain['sha256']
        pins[str(terrain_file)] = terrain['sha256']
        dataset = dict(cutoff_mm=cutoff, height_scale=height, terrain=terrain,
                       level=level(cutoff, height), poses={}, cells={}, rows={}, forward={}, failures={})
        for wave in WAVES:
            identity = (cutoff, height, wave)
            if identity not in cells:
                failure = failures.get(identity)
                assert failure is not None, f'Unaccounted selected observation: {identity}'
                dataset['failures'][wave] = failure
                continue
            cell = cells[identity]
            run = Path(cell['run_path'])
            environment = int(cell['environment_index'])
            if str(run) not in loaded:
                for path in [run / 'trajectory.npz', run / 'manifest.json', run / 'results.json', run / 'native_error_audit.json']:
                    pins[str(path)] = committed_hash(path)
                assert read(run / 'results.json')['complete']
                assert not read(run / 'native_error_audit.json')['errors']
                with np.load(run / 'trajectory.npz') as z:
                    now_names = z['body_names'].tolist()
                    assert names is None or now_names == names
                    names = now_names
                    t = z['t'].copy()
                    pose = z['pose'].copy()
                assert t.shape == (601,) and np.allclose(t, np.arange(601) * .01, atol=1e-12, rtol=0)
                assert t[-1] == 6. and pose.shape[0] == 601 and pose.shape[2:] == (251, 7)
                assert np.isfinite(pose).all()
                loaded[str(run)] = (pose, read(run / 'manifest.json')['rows'])
                source_receipts.append(dict(run_path=str(run), trajectory_sha256=pins[str(run / 'trajectory.npz')],
                                            manifest_sha256=pins[str(run / 'manifest.json')]))
            pose, rows = loaded[str(run)]
            row = rows[environment]
            assert row['wave'] == wave and row['height_scale'] == height
            assert float(row['cutoff_mm']) == cutoff and row['spawn_index'] == SPAWN
            assert row['surface_sha256'] == terrain['sha256']
            assert Path(row['surface_file']).resolve() == terrain_file.resolve()
            assert row['start_yaw_deg'] == campaign['spawns'][SPAWN]['start_yaw_deg']
            assert row['start_xy'] == campaign['spawns'][SPAWN]['start_xy_m']
            forward = series[key(cutoff, height, wave)]
            assert forward.shape == (601,) and np.isfinite(forward).all()
            assert abs(float(forward[-1]) - cell['forward_0_6_mm']) < 1e-7
            dataset['poses'][wave] = pose[:, environment]
            dataset['rows'][wave] = row
            dataset['cells'][wave] = cell
            dataset['forward'][wave] = forward
        datasets.append(dataset)
    return campaign, analysis, datasets, names, pins, source_receipts


def make_meshes(names):
    name_to_index = {name: i for i, name in enumerate(names)}
    radius = np.zeros(len(names))
    meshes = []
    for part in meta['parts']:
        poly, points = poly_for(part)
        body = name_to_index[part['body_name']]
        radius[body] = max(radius[body], float(np.linalg.norm(points, axis=1).max()))
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(poly)
        mapper.ScalarVisibilityOff()
        meshes.append((mapper, body, part))
    return meshes, radius


def camera_layout(campaign, datasets, spine, radius):
    yaw = float(campaign['spawns'][SPAWN]['start_yaw_deg'])
    theta = np.deg2rad(yaw - 90.)
    rotate = np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
    cameras = {'oblique': rotate @ np.array([.12, .028, .085]), 'top': np.array([0., 0., .2])}
    scales = {}
    for view, offset in cameras.items():
        forward = -offset / np.linalg.norm(offset)
        up0 = [0, 1, 0] if view == 'top' else [0, 0, 1]
        right = np.cross(forward, up0)
        right /= np.linalg.norm(right)
        up = np.cross(right, forward)
        half_height = .035
        for dataset in datasets:
            for pose in dataset['poses'].values():
                centers = pose[:, spine, :3].mean(1)
                rel = pose[:, :, :3] - centers[:, None, :]
                half_height = max(half_height, float((abs(rel @ up) + radius).max()) * 1.12,
                                  float((abs(rel @ right) + radius).max()) / 1.5 * 1.12)
        scales[view] = half_height
    return yaw, cameras, scales


def failure_tile(wave, failure):
    image = Image.new('RGB', (720, 480), '#eef0f3')
    draw = ImageDraw.Draw(image)
    draw.rectangle((16, 16, 703, 463), outline='#bd9292', width=2)
    status = str(failure.get('status', failure.get('failure_type', 'incomplete')))
    draw.text((35, 150), f'k={wave:g} · spawn {SPAWN}', font=titlefont, fill='#823535')
    draw.text((35, 205), '没有完整6 s轨迹；保留原失败', font=font, fill='#823535')
    draw.text((35, 249), status[:65], font=small, fill='#823535')
    draw.text((35, 290), '此格不以其他spawn替换，不赋零位移', font=small, fill='#526a7b')
    return image


def window_for(dataset, meshes, names, roi):
    ground, geometry = ground_mapper(next(iter(dataset['rows'].values())), roi)
    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetSize(1440, 1440)
    window.SetMultiSamples(4)
    bindings = {}
    actors = []
    for index, wave in enumerate(WAVES):
        if wave not in dataset['poses']:
            continue
        ren = vtk.vtkRenderer()
        row, col = divmod(index, 2)
        ren.SetViewport(col * .5, (2 - row) / 3, (col + 1) * .5, (3 - row) / 3)
        ren.SetBackground(.91, .94, .96)
        window.AddRenderer(ren)
        matrices = [vtk.vtkMatrix4x4() for _ in names]
        for mapper, body, part in meshes:
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.SetUserMatrix(matrices[body])
            linear = np.array(part['diffuse'])
            display = np.where(linear <= .0031308, 12.92 * linear, 1.055 * np.maximum(linear, 0)**(1 / 2.4) - .055)
            prop = actor.GetProperty()
            prop.SetColor(*display)
            prop.SetOpacity(part.get('opacity', 1.))
            prop.SetAmbient(.28)
            prop.SetDiffuse(.72)
            prop.SetSpecular(.32)
            prop.SetSpecularPower(22)
            ren.AddActor(actor)
            actors.append(actor)
        actor = vtk.vtkActor()
        actor.SetMapper(ground)
        actor.GetProperty().SetColor(.60, .57, .50)
        actor.GetProperty().SetAmbient(.28)
        actor.GetProperty().SetDiffuse(.72)
        ren.AddActor(actor)
        actors.append(actor)
        light = vtk.vtkLight()
        light.SetLightTypeToCameraLight()
        light.SetIntensity(.9)
        ren.AddLight(light)
        fill = vtk.vtkLight()
        fill.SetPosition(.1, .1, .2)
        fill.SetFocalPoint(0, 0, 0)
        fill.SetIntensity(.5)
        ren.AddLight(fill)
        bindings[wave] = (ren, matrices)
    capture = vtk.vtkWindowToImageFilter()
    capture.SetInput(window)
    capture.SetInputBufferTypeToRGB()
    capture.ReadFrontBufferOff()
    return window, capture, bindings, actors, ground, geometry


def canvas_at(dataset, index, view, yaw, cameras, scales, spine, radius, native):
    margin = None
    if native is not None:
        window, capture, bindings = native[:3]
        offset = cameras[view]
        half_height = scales[view]
        forward = -offset / np.linalg.norm(offset)
        up0 = [0, 1, 0] if view == 'top' else [0, 0, 1]
        right = np.cross(forward, up0)
        right /= np.linalg.norm(right)
        up = np.cross(right, forward)
        margin = 1.
        for wave, (ren, matrices) in bindings.items():
            pose = dataset['poses'][wave][index]
            for body, matrix in enumerate(matrices):
                rot = rotation(pose[body, 3:].astype(float))
                for a in range(3):
                    for b in range(3):
                        matrix.SetElement(a, b, rot[a, b])
                    matrix.SetElement(a, 3, float(pose[body, a]))
                matrix.SetElement(3, 3, 1.)
            center = pose[spine, :3].mean(0).astype(float)
            camera = ren.GetActiveCamera()
            camera.ParallelProjectionOn()
            camera.SetParallelScale(half_height)
            camera.SetViewUp(*up0)
            camera.SetClippingRange(.001, 2.)
            camera.SetPosition(*(center + offset))
            camera.SetFocalPoint(*center)
            rel = pose[:, :3] - center
            margin = min(margin, float((half_height - abs(rel @ up) - radius).min()),
                         float((half_height * 1.5 - abs(rel @ right) - radius).min()))
        window.Render()
        capture.Modified()
        capture.Update()
        pixels = vtk_to_numpy(capture.GetOutput().GetPointData().GetScalars()).reshape(1440, 1440, 3)[::-1]
    canvas = Image.new('RGB', (WIDTH, HEIGHT), '#f5f7fa')
    draw = ImageDraw.Draw(canvas)
    t = index * .01
    draw.text((22, 10), f"Libo · 截断{dataset['cutoff_mm']:g} mm · 高度×{dataset['height_scale']:g} · {view} | t={t:.2f} s", font=titlefont, fill='#142f45')
    draw.text((22, 53), '固定平地cluster A/B · 全173关节9/17 wave-4 PD · 0.5 ms物理 / 1 ms CPG', font=font, fill='#355169')
    draw.text((22, 87), f'实际Isaac CPU轨迹 · 原USD/VTK回放 · spawn {SPAWN} / yaw {yaw:.3f}° · 统一比例', font=small, fill='#526a7b')
    tiles = {}
    for number, wave in enumerate(WAVES):
        row, col = divmod(number, 2)
        x, y = col * 720, 125 + row * 535
        if wave in dataset['poses']:
            tile = Image.fromarray(pixels[row * 480:(row + 1) * 480, col * 720:(col + 1) * 720].copy())
            assert np.asarray(tile).std() > 5
            cell = dataset['cells'][wave]
            a, b = cell['A_deg'], cell['B_deg']
            displacement = dataset['forward'][wave][index]
            label = f'k={wave:g} | A={a:.3f}° / B={b:.3f}° | D={displacement:.2f} mm'
        else:
            tile = failure_tile(wave, dataset['failures'][wave])
            label = f'k={wave:g} | incomplete · no six-second displacement'
        draw.text((x + 18, y + 4), label, font=font, fill='#142f45')
        canvas.paste(tile, (x, y + 42))
        tiles[wave] = tile
    draw.text((740, 1290), '五个wave固定画格；失败不隐藏', font=font, fill='#526a7b')
    draw.text((740, 1340), f"固定足点roughness R={dataset['terrain']['fixed_pair_angle_rms_deg']:.3f}°", font=font, fill='#526a7b')
    draw.text((740, 1390), '地形垂直比例1倍；相机跟随身体', font=small, fill='#526a7b')
    draw.text((22, 1740), '保存轨迹回放；没有新增物理步。视频为1倍速20 fps，末帧5.95 s；静态图使用6.00 s。', font=small, fill='#526a7b')
    return canvas, tiles, margin


def decode_video(path, out, prefix):
    chosen = []
    with imageio.get_reader(path) as reader:
        metadata = reader.get_meta_data()
        count = 0
        for count, frame in enumerate(reader, 1):
            assert frame.shape == (HEIGHT, WIDTH, 3) and frame.std() > 5
            if count in (1, 61, 120):
                chosen.append(Image.fromarray(frame).resize((480, 587), Image.Resampling.LANCZOS))
            if count == 61:
                poster = out / f'{prefix}_poster.jpg'
                Image.fromarray(frame).save(poster, quality=90)
    assert count == 120 and metadata['fps'] == 20 and abs(metadata['duration'] - 6.) < .01
    strip = Image.new('RGB', (1440, 587), 'white')
    for index, frame in enumerate(chosen):
        strip.paste(frame, (480 * index, 0))
    review = out / f'{prefix}_decoded_review.jpg'
    strip.save(review, quality=90)
    return dict(path=str(path), sha256=sha(path), fps=20, frame_count=120, duration_s=6.,
                source_time_range_s=[0., 5.95], full_decode_passed=True,
                poster_path=str(poster), poster_sha256=sha(poster),
                first_mid_last_strip_path=str(review), first_mid_last_strip_sha256=sha(review),
                metadata={name: metadata[name] for name in ('fps', 'duration', 'size', 'codec', 'pix_fmt') if name in metadata},
                manually_reviewed=False)


def main(stage, video, out):
    started = time.time()
    assert re.fullmatch(r'[a-z0-9][a-z0-9_-]{0,63}', stage), 'Stage must identify one safe directory'
    base = EXPANSION / stage
    out = out.resolve() if out is not None else base / ('demos' if video else 'simulation_previews')
    assert not out.exists(), f'Preserve previous output; use another fresh --out: {out}'
    functions = load_graphics()
    campaign, analysis, datasets, names, pins, sources = load_stage(base)
    if names is None:
        names = [b['name'] for b in sorted(meta['bodies'], key=lambda b: b['index'])]
    assert len(names) == 251
    meshes, radius = make_meshes(names)
    spine = [i for i, name in enumerate(names) if name.startswith(('link_body_', 'link_pitch_body_'))]
    yaw, cameras, scales = camera_layout(campaign, datasets, spine, radius)
    out.mkdir(parents=True)
    report_path = out / ('VIDEO_REPORT.json' if video else 'RENDER_REPORT.json')
    receipt = dict(schema='libo_cutoff_height_spawn12_saved_trajectory_renderer_v1', complete=False,
                   stage=stage, video_mode=video, selected_spawn_index=SPAWN, selected_spawn_yaw_deg=yaw,
                   preselection='Spawn12 fixed for every terrain before performance inspection; incomplete cells never substituted.',
                   physics='Actual saved Isaac Lab CPU PhysX6s trajectories',
                   backend='VTK OpenGL rasterization of original USD visuals; not native RTX',
                   new_physics_steps_for_render=0, pid=os.getpid(), started_unix=started,
                   input_hashes={**{str(p): h for p, h in PINS.items()}, **pins}, sources=sources,
                   renderer_sha256=sha(__file__), function_receipt=functions,
                   geometry_sha256=sha(MAIN / 'usd_visual_geometry.npz'), asset_sha256=meta['asset_sha256'],
                   body_binding_count=len(names), visual_parts=len(meshes),
                   fixed_amplitudes=campaign['flat_cluster_cells'], fixed_pd_sha256=campaign['fixed_pd_sha256'],
                   camera_follows_spine_center=True, same_world_camera_orientation=True,
                   camera_offsets_world_m={name: value.tolist() for name, value in cameras.items()},
                   camera_half_heights_m=scales, shared_physical_scale_scope='All selected complete trajectories/terrains within this stage',
                   terrain_vertical_exaggeration=1., waves=WAVES, planned_selected_conditions=len(datasets) * 5,
                   complete_selected_conditions=sum(len(d['cells']) for d in datasets),
                   incomplete_selected_conditions=sum(len(d['failures']) for d in datasets),
                   figures=[], videos=[], no_video_terrains=[], ground_meshes={}, manually_reviewed=False)
    dump(report_path, receipt)
    mosaics = {}
    mosaic_columns = {}
    if not video:
        for cutoff in sorted({d['cutoff_mm'] for d in datasets}):
            members = [d for d in datasets if d['cutoff_mm'] == cutoff]
            mosaics[cutoff] = Image.new('RGB', (600 * len(members), 2400), '#f5f7fa')
            mosaic_columns[cutoff] = {d['level']: i for i, d in enumerate(members)}
            draw = ImageDraw.Draw(mosaics[cutoff])
            draw.text((22, 12), f'Libo · 截断{cutoff:g} mm · 五wave × 各高度 | 保存Isaac姿态 t=6.00 s', font=titlefont, fill='#142f45')
            draw.text((22, 55), f'spawn {SPAWN} / yaw {yaw:.3f}° · 固定A/B和全173关节PD · 统一视角与物理比例', font=font, fill='#355169')
    minimum_margin = None
    try:
        for dataset in datasets:
            prefix = f"libo_{dataset['level']}_five_waves_spawn12"
            native = window_for(dataset, meshes, names, campaign['terrain_roi_m']) if dataset['cells'] else None
            if native is not None:
                receipt['ground_meshes'][dataset['level']] = native[-1]
            try:
                if video and not dataset['cells']:
                    canvas, _, _ = canvas_at(dataset, 600, 'oblique', yaw, cameras, scales, spine, radius, None)
                    path = out / f'{prefix}_all_incomplete.png'
                    canvas.save(path)
                    receipt['figures'].append(dict(path=str(path), sha256=sha(path), time_s=6., all_incomplete=True, level=dataset['level']))
                    receipt['no_video_terrains'].append(dict(level=dataset['level'], reason='Allfive preselected spawn12 trajectories incomplete; no video synthesized', failures=list(dataset['failures'].values())))
                elif video:
                    path = out / f'{prefix}.mp4'
                    writer = imageio.get_writer(path, fps=20, codec='libx264', quality=8, macro_block_size=1,
                                                ffmpeg_params=['-threads', '2', '-movflags', '+faststart', '-preset', 'fast'])
                    try:
                        for frame in range(120):
                            canvas, _, margin = canvas_at(dataset, frame * 5, 'oblique', yaw, cameras, scales, spine, radius, native)
                            minimum_margin = margin if minimum_margin is None else min(minimum_margin, margin)
                            writer.append_data(np.asarray(canvas))
                            if frame % 40 == 0:
                                receipt['active_video'] = dict(level=dataset['level'], frames_encoded=frame + 1)
                                dump(report_path, receipt)
                                print(f"VIDEO {dataset['level']}: {frame + 1}/120", flush=True)
                    finally:
                        writer.close()
                    item = decode_video(path, out, prefix)
                    item.update(level=dataset['level'], cutoff_mm=dataset['cutoff_mm'], height_scale=dataset['height_scale'],
                                complete_waves=sorted(dataset['cells']), incomplete_waves=sorted(dataset['failures']), five_fixed_wave_tiles=True)
                    receipt['videos'].append(item)
                else:
                    for view in ('oblique', 'top'):
                        canvas, tiles, margin = canvas_at(dataset, 600, view, yaw, cameras, scales, spine, radius, native)
                        if margin is not None:
                            minimum_margin = margin if minimum_margin is None else min(minimum_margin, margin)
                        path = out / f'{prefix}_{view}_t6.png'
                        canvas.save(path)
                        with Image.open(path) as image:
                            image.load()
                        receipt['figures'].append(dict(path=str(path), sha256=sha(path), time_s=6., view=view,
                                                        level=dataset['level'], complete_waves=sorted(dataset['cells']),
                                                        incomplete_waves=sorted(dataset['failures']), image_decoded=True))
                        if view == 'oblique':
                            mosaic = mosaics[dataset['cutoff_mm']]
                            x = mosaic_columns[dataset['cutoff_mm']][dataset['level']] * 600
                            draw = ImageDraw.Draw(mosaic)
                            for row, wave in enumerate(WAVES):
                                y = 130 + row * 450
                                draw.text((x + 12, y), f"k={wave:g} · 高度×{dataset['height_scale']:g}", font=font, fill='#142f45')
                                mosaic.paste(tiles[wave].resize((600, 400), Image.Resampling.LANCZOS), (x, y + 38))
                dump(report_path, receipt)
            finally:
                if native is not None:
                    native[0].Finalize()
                    del native
                    gc.collect()
            print(f"Rendered saved poses: {dataset['level']} ({len(dataset['cells'])}/5 complete selected waves)", flush=True)
        for cutoff, mosaic in mosaics.items():
            path = out / f'libo_cutoff{cutoff:g}mm_all_terrains_t6.png'
            mosaic.save(path)
            receipt['figures'].append(dict(path=str(path), sha256=sha(path), cutoff_mm=cutoff,
                                            time_s=6., scientific_terrain_by_wave_mosaic=True))
        assert minimum_margin is None or minimum_margin > 0, 'Robot visuals clipped'
        for path, expected in receipt['input_hashes'].items():
            assert sha(path) == expected, f'Render input changed: {path}'
        shutil.copyfile(Path(__file__).resolve(), out / 'rendering_source.py')
        receipt.update(complete=True, elapsed_s=time.time() - started,
                       minimum_visual_frustum_margin_m=minimum_margin, no_native_physics=True,
                       executed_source_path=str(out / 'rendering_source.py'), executed_source_sha256=sha(out / 'rendering_source.py'))
        receipt.pop('active_video', None)
        dump(report_path, receipt)
        print(str(report_path), flush=True)
    except Exception as exc:
        receipt.update(complete=False, error_type=type(exc).__name__, error=str(exc), elapsed_s=time.time() - started)
        dump(report_path, receipt)
        raise
    finally:
        geo.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', required=True)
    parser.add_argument('--video', action='store_true')
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    main(args.stage, args.video, args.out)
