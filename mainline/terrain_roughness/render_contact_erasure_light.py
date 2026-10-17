"""Saved Isaac pose playback with physical collision masks highlighted. No physics."""
from pathlib import Path
import os,ast,json,hashlib,re,time,gc
for k in ['OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS']:os.environ[k]='1'
import numpy as np
ROOT=Path('X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness');MAIN=ROOT.parent
BASE=ROOT/'contact_erasure_light_2026-09-30';SIM=BASE/'simulation_v2';OUT=BASE/'demos'
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dump(p,d):Path(p).write_text(json.dumps(d,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')
parent=MAIN/'render_demo.py';source=parent.read_text(encoding='utf-8-sig');exec(source[:source.index('ap=argparse.ArgumentParser()')],globals())
for node in ast.parse(source).body:
    if isinstance(node,ast.FunctionDef) and node.name in ['poly_for','rotation']:exec(compile(ast.Module(body=[node],type_ignores=[]),str(parent),'exec'),globals())
from analyze_libo_repeats import extract_run
from PIL import Image,ImageDraw,ImageFont
WAVES=[1.5,2,2.5,3,3.5];BINS=[10,20,50]
OUT.mkdir(exist_ok=False);(OUT/'rendering_source.py').write_bytes(Path(__file__).read_bytes())
meta=read(MAIN/'usd_visual_geometry.json');geo=np.load(MAIN/'usd_visual_geometry.npz');assert sha(meta['asset'])==meta['asset_sha256']
datasets={};sources={}
for b in BINS:
    d=SIM/'runs/v1'/f'bin{b:03d}_p010_b00';ex=extract_run(d,10,90);cells=read(d/'amplitude_modulation.json')['cells'];assert [c['wave'] for c in cells[:5]]==WAVES and all(c['mask_seed']==0 for c in cells[:5])
    with np.load(d/'trajectory.npz') as z:poses=z['pose'][::5,:5].copy();names=z['body_names'].tolist()
    with np.load(d/'erasure_mask.npz') as z:masks=z['erased'][:,:5].copy()
    datasets[b]=dict(poses=poses,masks=masks,forward=ex['forward'][::5,:5]-ex['forward'][200,:5],cells=cells[:5],path=str(d))
    for f in ['trajectory.npz','erasure_mask.npz','amplitude_modulation.json']:sources[str(d/f)]=sha(d/f)
name_to_index={n:i for i,n in enumerate(names)};spine=[i for i,n in enumerate(names) if n.startswith(('link_body_','link_pitch_body_'))]
meshes=[];radii=np.zeros(251)
for part in meta['parts']:
    poly,points=poly_for(part);j=name_to_index[part['body_name']];radii[j]=max(radii[j],float(np.linalg.norm(points,axis=1).max()));mp=vtk.vtkPolyDataMapper();mp.SetInputData(poly);mp.ScalarVisibilityOff()
    n=part['body_name'];foot=re.fullmatch(r'foot_(\d+)_([01])',n);leg=re.fullmatch(r'link_leg_(\d+)_([LR])_[0-3]',n);channel=int(foot[1])*2+int(foot[2]) if foot else int(leg[1])*2+(leg[2]=='R') if leg else None
    color=np.array(part['diffuse']);color=np.where(color<=.0031308,12.92*color,1.055*np.maximum(color,0)**(1/2.4)-.055)
    meshes.append((mp,j,part,channel,color))
eyeoff=np.array([.14,-.035,.10]);forward=-eyeoff/np.linalg.norm(eyeoff);right=np.cross(forward,[0,0,1]);right/=np.linalg.norm(right);up=np.cross(right,forward)
hh=0.
for d in datasets.values():
    p=d['poses'];centers=p[:,:,:,:3][:,:,spine].mean(2);rel=p[:,:,:,:3]-centers[:,:,None,:]
    hh=max(hh,float((abs(rel@up)+radii).max()),float((abs(rel@right)+radii).max())/1.5)
hh*=1.1
font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',21);small=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',17);titlefont=ImageFont.truetype('C:/Windows/Fonts/msyhbd.ttc',28)
receipt=dict(complete=False,pid=os.getpid(),started_unix=time.time(),new_physics_steps=0,backend='VTK OpenGL rasterization of original USD visuals at actual saved Isaac poses',physics='Frozen latest CPU PhysX contact erasure; no new physics for rendering',p=.1,mask_seed=0,waves=WAVES,bins_ms=BINS,fps=20,playback_speed=1,source_frame_range_s=[0,5.95],mask_visualization='Pink translucent leg/foot geometry marks collision disabled; chassis stays active.20fps display aliases10/20ms switching, actual masks0.5ms saved.',sources=sources,renderer_sha256=sha(__file__),parent_renderer_sha256=sha(parent),geometry_sha256=sha(MAIN/'usd_visual_geometry.npz'),camera_halfheight_m=hh,videos=[])
for b,d in datasets.items():
    window=vtk.vtkRenderWindow();window.SetOffScreenRendering(1);window.SetSize(1440,1440);window.SetMultiSamples(4);renderers=[];matrices=[];actors=[]
    for e in range(5):
        ren=vtk.vtkRenderer();row,col=divmod(e,2);ren.SetViewport(col*.5,(2-row)/3,(col+1)*.5,(3-row)/3);ren.SetBackground(.91,.94,.96);window.AddRenderer(ren);renderers.append(ren);mats=[vtk.vtkMatrix4x4() for _ in names];matrices.append(mats);act=[]
        for mp,j,part,channel,color in meshes:
            a=vtk.vtkActor();a.SetMapper(mp);a.SetUserMatrix(mats[j]);pr=a.GetProperty();pr.SetColor(*color);pr.SetOpacity(part.get('opacity',1));pr.SetAmbient(.28);pr.SetDiffuse(.72);pr.SetSpecular(.28);pr.SetSpecularPower(20);ren.AddActor(a);act.append((a,channel,color,part.get('opacity',1)))
        actors.append(act);plane=vtk.vtkPlaneSource();plane.SetOrigin(-.3,-.43,0);plane.SetPoint1(.4,-.43,0);plane.SetPoint2(-.3,.2,0);plane.Update();mp=vtk.vtkPolyDataMapper();mp.SetInputConnection(plane.GetOutputPort());a=vtk.vtkActor();a.SetMapper(mp);a.GetProperty().SetColor(.64,.61,.56);ren.AddActor(a)
        light=vtk.vtkLight();light.SetLightTypeToCameraLight();light.SetIntensity(.9);ren.AddLight(light)
    capture=vtk.vtkWindowToImageFilter();capture.SetInput(window);capture.SetInputBufferTypeToRGB();capture.ReadFrontBufferOff();path=OUT/f'flat_p010_bin{b:03d}_seed00.mp4';writer=imageio.get_writer(path,fps=20,codec='libx264',quality=8,macro_block_size=1,ffmpeg_params=['-threads','2','-movflags','+faststart','-preset','fast'])
    for frame in range(120):
        t=frame/20;poses=d['poses'][frame];centers=poses[:,:,:3][:,spine].mean(1);mask=d['masks'][max(0,round(t/.0005)-1)]
        for e,ren in enumerate(renderers):
            for j,m in enumerate(matrices[e]):
                rot=rotation(poses[e,j,3:].astype(float))
                for aa in range(3):
                    for bb in range(3):m.SetElement(aa,bb,rot[aa,bb])
                    m.SetElement(aa,3,float(poses[e,j,aa]))
                m.SetElement(3,3,1.)
            for actor,channel,color,opacity in actors[e]:
                erased=channel is not None and mask[e,channel];pr=actor.GetProperty();pr.SetColor(*(np.array([.98,.30,.56]) if erased else color));pr.SetOpacity(.40 if erased else opacity)
            center=centers[e].astype(float);cam=ren.GetActiveCamera();cam.ParallelProjectionOn();cam.SetParallelScale(hh);cam.SetViewUp(0,0,1);cam.SetClippingRange(.001,2);cam.SetPosition(*(center+eyeoff));cam.SetFocalPoint(*center)
        window.Render();capture.Modified();capture.Update();pixels=vtk_to_numpy(capture.GetOutput().GetPointData().GetScalars()).reshape(1440,1440,3)[::-1]
        canvas=Image.new('RGB',(1440,1760),'#f5f7fa');draw=ImageDraw.Draw(canvas);draw.text((22,10),f'平地 contact erasure · p=0.1 · hold={b} ms · seed0 · t={t:.2f}s',font=titlefont,fill='#142f45');draw.text((22,52),'实际 Isaac CPU PhysX 轨迹 · 固定 cluster A/B、9/17 wave4完整PD · 0.5ms物理 / 1ms CPG',font=font,fill='#355169');draw.text((22,87),'0–2s不擦除，2–6s计分；粉色半透明腿：此刻碰撞关闭，身体碰撞保留。',font=font,fill='#526a7b')
        for e,cell in enumerate(d['cells']):
            row,col=divmod(e,2);tile=Image.fromarray(pixels[row*480:(row+1)*480,col*720:(col+1)*720].copy());assert np.asarray(tile).std()>5;x=col*720;y=125+row*535;score=d['forward'][frame,e] if t>=2 else 0.
            draw.text((x+18,y+4),f'k={cell["wave"]:g} | D(2–t)={score:.2f}mm | erased={int(mask[e].sum())}/38',font=font,fill='#142f45');canvas.paste(tile,(x,y+42))
        draw.text((750,1230),'38条腿左右独立\n跨wave共享时间mask\n实际擦除状态按所选hold更新\n20fps画面不能逐一显示10ms切换\n原USD视觉模型 OpenGL回放\n相机跟随躯干，所有wave同一比例',font=font,fill='#355169',spacing=18)
        writer.append_data(np.asarray(canvas))
        if frame==60:canvas.save(OUT/f'bin{b:03d}_poster.jpg',quality=92)
    writer.close();count=0
    with imageio.get_reader(path) as reader:
        md=reader.get_meta_data()
        for count,pixels in enumerate(reader,1):assert pixels.shape==(1760,1440,3) and pixels.std()>5
    assert count==120 and abs(md['duration']-6)<.02
    receipt['videos'].append(dict(path=str(path),sha256=sha(path),full_decode_passed=True,frames=count,duration_s=6,bin_ms=b,waves=WAVES));dump(OUT/'VIDEO_REPORT.json',receipt);window.Finalize();del window,renderers,matrices,actors,capture;gc.collect();print(f'Rendered saved Isaac5wave erasure demo {b}ms',flush=True)
receipt['complete']=True;receipt['wall_s']=time.time()-receipt['started_unix'];dump(OUT/'VIDEO_REPORT.json',receipt)
