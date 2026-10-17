"""Render original USD visuals at saved six-second Isaac preview poses; no physics."""
from pathlib import Path
import os
for k in ['OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS']:os.environ[k]='1'
import ast,json,hashlib,time,types,gc
import numpy as np
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parent;MAIN=ROOT.parent;SIM=ROOT/'height_amplitude_4mm/simulation';OUT=ROOT/'height_amplitude_4mm/simulation_previews'
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dump(p,d):Path(p).write_text(json.dumps(d,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')
# Reuse only the established renderer's imports and geometry/rotation functions.
parent=MAIN/'render_demo.py';source=parent.read_text(encoding="utf-8-sig");exec(source[:source.index('ap=argparse.ArgumentParser()')],globals())
tree=ast.parse(source)
for node in tree.body:
    if isinstance(node,ast.FunctionDef) and node.name in ['poly_for','rotation']:
        exec(compile(ast.Module(body=[node],type_ignores=[]),str(parent),'exec'),globals())
from vtkmodules.util.numpy_support import numpy_to_vtkIdTypeArray

SCALES=[.25,.5,.75,1.,1.25,1.5,2.]
LEVELS=['height_x'+f'{x:.2f}'.replace('.','p') for x in SCALES];LABELS={level:f'高度 ×{scale:g} · 截断4 mm' for level,scale in zip(LEVELS,SCALES)}
font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',21);small=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',17);titlefont=ImageFont.truetype('C:/Windows/Fonts/msyhbd.ttc',28)

def ground_mapper(row,roi):
    path=Path(row['surface_file']);assert sha(path)==row['surface_sha256']
    with np.load(path) as z:h=z['height'];dx=float(z['dx_m'])
    ay=(np.arange(h.shape[0])-(h.shape[0]-1)/2)*dx;ax=(np.arange(h.shape[1])-(h.shape[1]-1)/2)*dx
    ix=np.flatnonzero((ax>=roi[0]-1e-10)&(ax<=roi[1]+1e-10));iy=np.flatnonzero((ay>=roi[2]-1e-10)&(ay<=roi[3]+1e-10));h=h[np.ix_(iy,ix)];xx,yy=np.meshgrid(ax[ix],ay[iy]);ny,nx=h.shape
    points=np.c_[xx.ravel(),yy.ravel(),h.ravel()].astype(np.float32);rr,cc=np.meshgrid(np.arange(ny-1),np.arange(nx-1),indexing='ij');st=(rr*nx+cc).ravel();fs=np.vstack([np.c_[st,st+1,st+nx+1],np.c_[st,st+nx+1,st+nx]]).astype(np.int64)
    poly=vtk.vtkPolyData();vp=vtk.vtkPoints();vp.SetData(numpy_to_vtk(points,deep=True));poly.SetPoints(vp);ca=vtk.vtkCellArray();ca.SetCells(len(fs),numpy_to_vtkIdTypeArray(np.c_[np.full(len(fs),3),fs].ravel(),deep=True));poly.SetPolys(ca)
    normals=vtk.vtkPolyDataNormals();normals.SetInputData(poly);normals.SplittingOff();normals.Update();mp=vtk.vtkPolyDataMapper();mp.SetInputConnection(normals.GetOutputPort());mp.ScalarVisibilityOff()
    return mp,dict(vertices=len(points),triangles=len(fs),vertex_sha256=hashlib.sha256(points.tobytes()).hexdigest(),float32_collision_geometry=True,display_normals_smoothed=True)

def main(attempt,video=False):
    global meta,geo,OUT
    if video:
        OUT=ROOT/"height_amplitude_4mm/demos"
        if OUT.exists() and read(OUT/"VIDEO_REPORT.json").get("complete"):raise RuntimeError("Completed videos exist; do not overwrite")
    previous=read(OUT/"VIDEO_REPORT.json") if video and (OUT/"VIDEO_REPORT.json").exists() else None
    started=time.time()
    analysis=read(ROOT/'height_amplitude_4mm/results/RESULTS.json');assert analysis['complete']
    resolution=analysis['run_resolution']
    c=read(SIM/'campaign.json');OUT.mkdir(exist_ok=True)
    meta=read(MAIN/'usd_visual_geometry.json');geo=np.load(MAIN/'usd_visual_geometry.npz');assert sha(meta['asset'])==meta['asset_sha256']
    datasets={};name_order=None;source_receipts=[]
    for site in ['libo','narvik']:
        for level in LEVELS:
            arrays=[];cells=[];rows=[];missing=[]
            for group in ['low','high']:
                selected=resolution[f'{site}_{level}_{group}']
                if not selected['complete']:
                    missing.extend([1.5,2,2.5] if group=='low' else [3,3.5,4]);continue
                d=Path(selected['path']);result=read(d/'results.json');assert result['complete'] and result['actual_duration_s']==6 and result['whole_batch_finite'];assert not read(d/'native_error_audit.json')['errors'];assert read(d/'crop_guard.json')['passed']
                with np.load(d/'trajectory.npz') as z:
                    assert z['t'][-1]==6. and z['pose'].shape==(601,3,251,7);assert np.isfinite(z['pose']).all()
                    names=z['body_names'].tolist();assert name_order is None or names==name_order;name_order=names
                    arrays.append(z['pose'][:600:5].copy() if video else z['pose'][[450,600]].copy())
                cells+=read(d/'amplitude_modulation.json')['cells'];rows+=read(d/'manifest.json')['rows'];source_receipts.append(dict(run=str(d),trajectory_sha256=sha(d/'trajectory.npz')))
            datasets[(site,level)]=dict(poses=np.concatenate(arrays,axis=1),cells=cells,rows=rows,missing=missing)
    names=name_order;name_to_index={n:i for i,n in enumerate(names)};spine=[j for j,n in enumerate(names) if n.startswith(('link_body_','link_pitch_body_'))]
    meshes=[];radius=np.zeros(len(names))
    for part in meta['parts']:
        poly,points=poly_for(part);j=name_to_index[part['body_name']];radius[j]=max(radius[j],float(np.linalg.norm(points,axis=1).max()));mp=vtk.vtkPolyDataMapper();mp.SetInputData(poly);mp.ScalarVisibilityOff();meshes.append((mp,j,part))
    cameras={'oblique':np.array([.028,-.12,.085]),'top':np.array([0.,0.,.2])};halfheights={}
    for view,eyeoff in cameras.items():
        forward=-eyeoff/np.linalg.norm(eyeoff);up0=[0,1,0] if view=='top' else [0,0,1];right=np.cross(forward,up0);right/=np.linalg.norm(right);up=np.cross(right,forward);hh=.035
        for d in datasets.values():
            poses=d['poses'];centers=poses[:,:,:,:3][:,:,spine].mean(2);rel=poses[:,:,:,:3]-centers[:,:,None,:]
            hh=max(hh,float((abs(rel@up)+radius).max())*1.12,float((abs(rel@right)+radius).max())/1.5*1.12)
        halfheights[view]=hh
    receipt=dict(complete=False,attempts=analysis['attempts'],incomplete_cells=analysis['incomplete_cells'],physics='Actual saved Isaac Lab CPU PhysX, 6 s per condition',backend='VTK OpenGL rasterization of original USD visuals; not native RTX',new_physics_steps_for_render=0,
        sources=source_receipts,renderer_sha256=sha(__file__),parent_renderer_sha256=sha(parent),geometry_sha256=sha(MAIN/'usd_visual_geometry.npz'),asset_sha256=meta['asset_sha256'],camera_half_heights_m=halfheights,
        fixed_amplitudes=c['flat_cluster_cells'],fixed_pd_sha256=c['fixed_pd_sha256'],camera_follows_spine_center=True,same_world_camera_orientation=True,terrain_vertical_exaggeration=1.,figures=[],videos=[],ground_meshes={},video_mode=video,pid=os.getpid(),started_unix=started,
        fps=20 if video else None,playback_speed=1. if video else None,source_frame_time_range_s=[0.,5.95] if video else [4.5,6.])
    if previous:assert previous['sources']==receipt['sources'] and previous['fixed_pd_sha256']==receipt['fixed_pd_sha256'],'Input changes forbid reuse'
    if video:dump(OUT/'VIDEO_REPORT.json',receipt)
    min_margin=1.;mosaics={} if video else {s:Image.new('RGB',(600*len(LEVELS),2850),'#f5f7fa') for s in ['libo','narvik']}
    for site,m in mosaics.items():
        draw=ImageDraw.Draw(m);draw.text((22,10),f'{site.title()} · 六波数 × 七档高度 | 实际 Isaac 物理姿态 t=6.00 s',font=titlefont,fill='#142f45')
        draw.text((22,53),'固定平地 cluster A/B · 全173关节 9/17 wave-4 PD · 0.5 ms物理 / 1 ms CPG',font=font,fill='#355169')
        draw.text((22,87),'原 USD 模型 OpenGL 回放；统一视角和比例。短时预览，不代表完整性能验证。',font=small,fill='#526a7b')
    for (site,level),d in datasets.items():
        ground,ginfo=ground_mapper(d['rows'][0],c['terrain_roi_m']);receipt['ground_meshes'][site+'_'+level]=ginfo
        suffix='six_waves' if len(d['cells'])==6 else 'high_waves'
        existing_video=OUT/f'{site}_{level}_{suffix}.mp4'
        if video and existing_video.exists():
            assert previous is not None
            with imageio.get_reader(existing_video) as reader:
                metadata=reader.get_meta_data();count=0
                for count,frame in enumerate(reader,1):assert frame.shape==(1760,1440,3) and frame.std()>5
            assert count==120 and metadata['fps']==20 and abs(metadata['duration']-6.)<.01
            receipt['videos'].append(dict(path=str(existing_video),sha256=sha(existing_video),site=site,level=level,fps=20,frame_count=120,duration_s=6.,source_time_range_s=[0.,5.95],full_decode_passed=True,reused_after_metadata_only_failure=True,renderer_source_sha256=previous['renderer_sha256'],metadata={k:metadata[k] for k in ['fps','duration','size','codec','pix_fmt'] if k in metadata},waves=[x['wave'] for x in d['cells']]))
            dump(OUT/'VIDEO_REPORT.json',receipt);print('Verified and reused completed video: '+site+' '+level,flush=True)
            continue
        window=vtk.vtkRenderWindow();window.SetOffScreenRendering(1);window.SetSize(1440,1440);window.SetMultiSamples(4);renderers=[];matrices=[];actors=[]
        for e in range(len(d['cells'])):
            ren=vtk.vtkRenderer();row,col=divmod(e,2);ren.SetViewport(col*.5,(2-row)/3,(col+1)*.5,(3-row)/3);ren.SetBackground(.91,.94,.96);window.AddRenderer(ren);renderers.append(ren)
            mats=[vtk.vtkMatrix4x4() for _ in names];matrices.append(mats)
            for mp,j,part in meshes:
                color=np.array(part['diffuse']);color=np.where(color<=.0031308,12.92*color,1.055*np.maximum(color,0)**(1/2.4)-.055)
                a=vtk.vtkActor();a.SetMapper(mp);a.SetUserMatrix(mats[j]);prop=a.GetProperty();prop.SetColor(*color);prop.SetOpacity(part.get('opacity',1));prop.SetAmbient(.28);prop.SetDiffuse(.72);prop.SetSpecular(.32);prop.SetSpecularPower(22);ren.AddActor(a);actors.append(a)
            a=vtk.vtkActor();a.SetMapper(ground);a.GetProperty().SetColor(.60,.57,.50);a.GetProperty().SetAmbient(.28);a.GetProperty().SetDiffuse(.72);ren.AddActor(a);actors.append(a)
            light=vtk.vtkLight();light.SetLightTypeToCameraLight();light.SetIntensity(.9);ren.AddLight(light);fill=vtk.vtkLight();fill.SetPosition(.1,.1,.2);fill.SetFocalPoint(0,0,0);fill.SetIntensity(.5);ren.AddLight(fill)
        capture=vtk.vtkWindowToImageFilter();capture.SetInput(window);capture.SetInputBufferTypeToRGB();capture.ReadFrontBufferOff()
        video_path=OUT/f'{site}_{level}_{suffix}.mp4'
        writer=imageio.get_writer(video_path,fps=20,codec='libx264',quality=8,macro_block_size=1,ffmpeg_params=['-threads','2','-movflags','+faststart','-preset','fast']) if video else None
        sequence=[('oblique',i) for i in range(120)] if video else [('oblique',0),('oblique',1),('top',1)]
        for view,time_index in sequence:
            t=time_index/20 if video else [4.5,6.][time_index];poses=d['poses'][time_index];centers=poses[:,:,:3][:,spine].mean(1);eyeoff=cameras[view];hh=halfheights[view]
            forward=-eyeoff/np.linalg.norm(eyeoff);up0=[0,1,0] if view=='top' else [0,0,1];right=np.cross(forward,up0);right/=np.linalg.norm(right);up=np.cross(right,forward)
            for e,ren in enumerate(renderers):
                for j,matrix in enumerate(matrices[e]):
                    rot=rotation(poses[e,j,3:].astype(float))
                    for a in range(3):
                        for b in range(3):matrix.SetElement(a,b,rot[a,b])
                        matrix.SetElement(a,3,float(poses[e,j,a]))
                    matrix.SetElement(3,3,1.)
                center=centers[e].astype(float);cam=ren.GetActiveCamera();cam.ParallelProjectionOn();cam.SetParallelScale(hh);cam.SetViewUp(*up0);cam.SetClippingRange(.001,2);cam.SetPosition(*(center+eyeoff));cam.SetFocalPoint(*center)
                rel=poses[e,:,:3]-center;min_margin=min(min_margin,float((hh-abs(rel@up)-radius).min()),float((hh*1.5-abs(rel@right)-radius).min()))
            window.Render();capture.Modified();capture.Update();pixels=vtk_to_numpy(capture.GetOutput().GetPointData().GetScalars()).reshape(1440,1440,3)[::-1]
            canvas=Image.new('RGB',(1440,1760),'#f5f7fa');draw=ImageDraw.Draw(canvas);draw.text((22,10),f'{site.title()} · {LABELS[level]} · '+('俯视' if view=='top' else '斜视')+f' | t={t:.2f} s',font=titlefont,fill='#142f45')
            draw.text((22,53),'固定平地 cluster A/B · 9/17 wave-4全关节PD · 0.5 ms物理 / 1 ms CPG',font=font,fill='#355169');draw.text((22,87),'实际 Isaac CPU PhysX 轨迹 · 原 USD 模型 OpenGL 回放 · 共用相机比例'+(' · 1倍速 / 20 fps' if video else ''),font=small,fill='#526a7b')
            for e,cell in enumerate(d['cells']):
                row,col=divmod(e,2);tile=Image.fromarray(pixels[row*480:(row+1)*480,col*720:(col+1)*720].copy());assert np.asarray(tile).std()>5
                x=col*720;y=125+row*535;draw.text((x+18,y+4),f'k={cell["wave"]:g} | A={cell["A_deg"]:.3f}° / B={cell["B_deg"]:.3f}°',font=font,fill='#142f45');canvas.paste(tile,(x,y+42))
                if not video and view=='oblique' and t==6:
                    m=mosaics[site];md=ImageDraw.Draw(m);mx=LEVELS.index(level)*600;my=135+e*450;md.text((mx+13,my),f'k={cell["wave"]:g} · {LABELS[level]}',font=font,fill='#142f45');m.paste(tile.resize((600,400),Image.Resampling.LANCZOS),(mx,my+38))
            if d['missing']:
                draw.text((22,1740), 'No complete 6 s record for k=1.5, 2, 2.5: shared worker stopped at a divergent state.',font=small,fill='#9a352e')
            if video:
                writer.append_data(np.asarray(canvas))
                if time_index%40==0:
                    print(f'VIDEO {site} {level}: {time_index}/120 frames',flush=True)
                    receipt['active_video']=dict(site=site,level=level,frames_encoded=time_index+1)
                    dump(OUT/'VIDEO_REPORT.json',receipt)
            else:
                filename=f'{site}_{level}_{view}_t{t:g}.png';canvas.save(OUT/filename);receipt['figures'].append(dict(path=str(OUT/filename),sha256=sha(OUT/filename),site=site,level=level,time_s=t,view=view,waves=[x['wave'] for x in d['cells']]))
        if video:
            writer.close()
            selected_frames=[]
            with imageio.get_reader(video_path) as reader:
                metadata=reader.get_meta_data();count=0
                for count,decoded in enumerate(reader,1):
                    assert decoded.shape==(1760,1440,3) and decoded.std()>5
                    if count in [1,61,120]:selected_frames.append(Image.fromarray(decoded).resize((480,587),Image.Resampling.LANCZOS))
                    if count==61:Image.fromarray(decoded).save(OUT/f'{site}_{level}_poster.jpg',quality=90)
            assert count==120 and metadata['fps']==20 and abs(metadata['duration']-6.)<.01
            review=Image.new('RGB',(1440,587),'white')
            for k,im in enumerate(selected_frames):review.paste(im,(k*480,0))
            review.save(OUT/f'{site}_{level}_decoded_review.jpg',quality=90)
            receipt['videos'].append(dict(path=str(video_path),sha256=sha(video_path),site=site,level=level,fps=20,frame_count=120,duration_s=6.,source_time_range_s=[0.,5.95],full_decode_passed=True,renderer_source_sha256=sha(__file__),metadata={k:metadata[k] for k in ['fps','duration','size','codec','pix_fmt'] if k in metadata},waves=[x['wave'] for x in d['cells']]))
            dump(OUT/'VIDEO_REPORT.json',receipt)
        window.Finalize();del window,renderers,matrices,actors,ground,capture;gc.collect();print('Rendered Isaac poses: '+site+' '+level,flush=True)
    for site,m in mosaics.items():m.save(OUT/f'{site}_all_waves_all_terrains.png')
    assert min_margin>0;receipt.update(complete=True,minimum_visual_frustum_margin_m=min_margin,body_binding_count=len(names),visual_parts=len(meshes),condition_count=sum(len(d['cells']) for d in datasets.values()),planned_condition_count=84,manually_reviewed=False)
    receipt['elapsed_s']=time.time()-started
    receipt.pop('active_video',None)
    dump(OUT/('VIDEO_REPORT.json' if video else 'RENDER_REPORT.json'),receipt);print('All simulation demo videos saved' if video else 'All simulation preview images saved',flush=True)
if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--attempt',default='v1');p.add_argument('--video',action='store_true');a=p.parse_args();main(a.attempt,a.video)

