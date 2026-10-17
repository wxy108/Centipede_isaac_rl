"""Render original USD visual geometry at recorded Isaac poses via VTK OpenGL.

This is graphics only. No physics integrator, controller, or optimization is run.
RTX initialization failed in the installed native Isaac renderer; this backend
is explicitly labeled on all frames and in provenance.
"""
import os
for name in ('OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS'):
    os.environ[name]='1'
import argparse,json,time,hashlib
from pathlib import Path
import numpy as np
import types
from vtkmodules.vtkCommonCore import vtkPoints
from vtkmodules.vtkCommonDataModel import vtkPolyData,vtkCellArray
from vtkmodules.vtkCommonMath import vtkMatrix4x4
from vtkmodules.vtkFiltersSources import vtkSphereSource,vtkCylinderSource,vtkCubeSource,vtkPlaneSource,vtkLineSource
from vtkmodules.vtkFiltersCore import vtkTriangleFilter,vtkPolyDataNormals,vtkAppendPolyData
from vtkmodules.vtkRenderingCore import vtkRenderWindow,vtkRenderer,vtkPolyDataMapper,vtkActor,vtkLight,vtkWindowToImageFilter
import vtkmodules.vtkRenderingOpenGL2
from vtkmodules.util.numpy_support import numpy_to_vtk,vtk_to_numpy
vtk=types.SimpleNamespace(**{k:v for k,v in list(globals().items()) if k.startswith('vtk')})
import imageio.v2 as imageio
from PIL import Image,ImageDraw,ImageFont

ap=argparse.ArgumentParser();ap.add_argument('--terrain',choices=['flat','rough'],required=True);ap.add_argument('--run-attempt',default='v3');ap.add_argument('--preview',action='store_true');args=ap.parse_args()
W=Path(__file__).resolve().parent;R=W.parents[1];O=R/'outputs/unified_demo_2026-09-28';D=O/(args.terrain+('_preview' if args.preview else '_demo'))
D.mkdir(parents=True,exist_ok=False)
def read(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
meta=read(W/'usd_visual_geometry.json');geo=np.load(W/'usd_visual_geometry.npz')
assert meta['asset_sha256']==sha(Path(meta['asset']))
sources=[W/'runs'/args.run_attempt/(args.terrain+'_'+group) for group in ['low','high']]
datasets=[np.load(p/'trajectory.npz') for p in sources]
poses=np.concatenate([z['pose'] for z in datasets],axis=1);times=datasets[0]['t'];names=datasets[0]['body_names'].tolist()
for z in datasets:assert z['body_names'].tolist()==names and np.array_equal(z['t'],times)
assert np.isfinite(poses).all() and poses.shape==(1201,6,251,7) and times[-1]==12.
name_to_index={n:i for i,n in enumerate(names)}
cells=sum([read(p/'amplitude_modulation.json')['cells'] for p in sources],[])
rows=sum([read(p/'manifest.json')['rows'] for p in sources],[])
spine=[j for j,n in enumerate(names) if n.startswith(('link_body_','link_pitch_body_'))]
centers=poses[:,:,:,:3][:,:,spine,:].mean(2)
receipt=dict(terrain=args.terrain,backend='VTK OpenGL rasterization of original USD visual geometry',physics='New recorded Isaac CPU PhysX, unified PD 2026-09-28',rendering_physics_steps=0,sources=[str(p) for p in sources],source_pose_sha256=[sha(p/'trajectory.npz') for p in sources],surface_sha256=[r['surface_sha256'] for r in rows],geometry_sha256=sha(W/'usd_visual_geometry.npz'),complete=False,pid=os.getpid(),started_unix=time.time(),frames_completed=0,preview=args.preview)
def save(): (D/'receipt.json').write_text(json.dumps(receipt,indent=2),encoding='utf8')
save()

def poly_for(part):
    if part['type']=='Mesh':
        points=np.array(geo[part['points_body_key']],dtype=float)
        fs=geo[part['faces_key']].astype(np.int64);poly=vtk.vtkPolyData();vp=vtk.vtkPoints();vp.SetData(numpy_to_vtk(points,deep=True));poly.SetPoints(vp)
        ca=vtk.vtkCellArray();j=0
        while j<len(fs):
            n=int(fs[j]);ca.InsertNextCell(n)
            for v in fs[j+1:j+1+n]:ca.InsertCellPoint(int(v))
            j+=n+1
        poly.SetPolys(ca)
    else:
        kind=part['type']
        if kind=='Sphere':
            src=vtk.vtkSphereSource();src.SetRadius(part['radius']);src.SetThetaResolution(24);src.SetPhiResolution(16)
        elif kind in ('Cylinder','Capsule'):
            src=vtk.vtkCylinderSource();src.SetRadius(part['radius']);src.SetHeight(part['height']);src.SetResolution(24);src.CappingOn()
            if kind=='Capsule':src.CapsuleCapOn()
        elif kind=='Cube':src=vtk.vtkCubeSource();src.SetXLength(part['size']);src.SetYLength(part['size']);src.SetZLength(part['size'])
        else:raise ValueError(('Unhandled primitive',kind,part))
        src.Update();poly=vtk.vtkPolyData();poly.DeepCopy(src.GetOutput())
        points=vtk_to_numpy(poly.GetPoints().GetData()).astype(float)
        if kind in ('Cylinder','Capsule'):
            # VTK cylinder is Y-axis; USD primitive axis is explicit.
            axis=part.get('axis','Z')
            if axis=='Z':points=points[:,[0,2,1]];points[:,1]*=-1
            elif axis=='X':points=points[:,[1,0,2]];points[:,1]*=-1
        mat=np.array(part['local_to_body_matrix_row'],float)
        points=(np.c_[points,np.ones(len(points))]@mat)[:,:3]
        vp=vtk.vtkPoints();vp.SetData(numpy_to_vtk(points,deep=True));poly.SetPoints(vp)
    tri=vtk.vtkTriangleFilter();tri.SetInputData(poly);tri.Update()
    normals=vtk.vtkPolyDataNormals();normals.SetInputConnection(tri.GetOutputPort());normals.ConsistencyOn();normals.SplittingOn();normals.SetFeatureAngle(65);normals.Update()
    out=vtk.vtkPolyData();out.DeepCopy(normals.GetOutput());return out,points

meshes=[];radius=np.zeros(len(names));sourceparts=[]
for part in meta['parts']:
    poly,points=poly_for(part);j=name_to_index[part['body_name']]
    radius[j]=max(radius[j],float(np.linalg.norm(points,axis=1).max()))
    mapper=vtk.vtkPolyDataMapper();mapper.SetInputData(poly);mapper.ScalarVisibilityOff()
    meshes.append((mapper,j,part));sourceparts.append((part['source_prim'],len(points)))
assert len(meshes)>200
eyeoff=np.array([.028,-.12,.085]);forward=-eyeoff/np.linalg.norm(eyeoff);right=np.cross(forward,[0,0,1]);right/=np.linalg.norm(right);up=np.cross(right,forward)
rel=poses[:,:,:,:3].astype(float)-centers[:,:,None,:]
halfheight=max(float((abs(rel@up)+radius[None,None,:]).max()),float((abs(rel@right)+radius[None,None,:]).max())/1.5)*1.1
other='rough' if args.terrain=='flat' else 'flat'
otherposes=np.concatenate([np.load(W/'runs'/args.run_attempt/(other+'_'+group)/'trajectory.npz')['pose'] for group in ['low','high']],axis=1)
othercenters=otherposes[:,:,:,:3][:,:,spine,:].mean(2)
otherrel=otherposes[:,:,:,:3].astype(float)-othercenters[:,:,None,:]
halfheight=max(halfheight,max(float((abs(otherrel@up)+radius[None,None,:]).max()),float((abs(otherrel@right)+radius[None,None,:]).max())/1.5)*1.1)

window=vtk.vtkRenderWindow();window.SetOffScreenRendering(1);window.SetSize(1440,1440);window.SetMultiSamples(4)
renderers=[];matrices=[];actorrefs=[]
for e in range(6):
    ren=vtk.vtkRenderer();row,col=divmod(e,2);ren.SetViewport(col*.5,(2-row)/3,(col+1)*.5,(3-row)/3)
    ren.SetBackground(.91,.94,.96);window.AddRenderer(ren);renderers.append(ren)
    mats=[vtk.vtkMatrix4x4() for _ in names];matrices.append(mats)
    for mapper,j,part in meshes:
        # USD constant colors are linear. Encode sRGB for this raster preview.
        linear=np.array(part['diffuse']);display=np.where(linear<=.0031308,12.92*linear,1.055*np.maximum(linear,0)**(1/2.4)-.055)
        actor=vtk.vtkActor();actor.SetMapper(mapper);actor.SetUserMatrix(mats[j]);pr=actor.GetProperty();pr.SetColor(*display);pr.SetOpacity(part.get('opacity',1.));pr.SetAmbient(.28);pr.SetDiffuse(.72);pr.SetSpecular(.32);pr.SetSpecularPower(22)
        ren.AddActor(actor);actorrefs.append(actor)
    if args.terrain=='rough':
        surface=Path(rows[e]['surface_file']);assert sha(surface)==rows[e]['surface_sha256']
        z=np.load(surface);h=z['height'];dx=float(z['dx_m']);ny,nx=h.shape
        xx,yy=np.meshgrid((np.arange(nx)-(nx-1)/2)*dx,(np.arange(ny)-(ny-1)/2)*dx)
        points=np.c_[xx.ravel(),yy.ravel(),h.ravel()].astype(np.float32)
        rr,cc=np.meshgrid(np.arange(ny-1),np.arange(nx-1),indexing='ij');st=(rr*nx+cc).ravel()
        fs=np.vstack([np.c_[st,st+1,st+nx+1],np.c_[st,st+nx+1,st+nx]]).astype(np.int64)
        poly=vtk.vtkPolyData();vp=vtk.vtkPoints();vp.SetData(numpy_to_vtk(points,deep=True));poly.SetPoints(vp)
        from vtkmodules.util.numpy_support import numpy_to_vtkIdTypeArray
        ca=vtk.vtkCellArray();ca.SetCells(len(fs),numpy_to_vtkIdTypeArray(np.c_[np.full(len(fs),3),fs].ravel(),deep=True));poly.SetPolys(ca)
        norms=vtk.vtkPolyDataNormals();norms.SetInputData(poly);norms.SplittingOff();norms.Update()
        mp=vtk.vtkPolyDataMapper();mp.SetInputConnection(norms.GetOutputPort());mp.ScalarVisibilityOff()
        a=vtk.vtkActor();a.SetMapper(mp);a.GetProperty().SetColor(.60,.57,.50);a.GetProperty().SetAmbient(.28);a.GetProperty().SetDiffuse(.72);ren.AddActor(a)
    else:
        plane=vtk.vtkPlaneSource();plane.SetOrigin(-1,-1,-.00002);plane.SetPoint1(1,-1,-.00002);plane.SetPoint2(-1,1,-.00002)
        mp=vtk.vtkPolyDataMapper();mp.SetInputConnection(plane.GetOutputPort());a=vtk.vtkActor();a.SetMapper(mp);a.GetProperty().SetColor(.88,.91,.93);a.GetProperty().LightingOff();ren.AddActor(a)
        # Physical ground grid is a visual reference only, at 10 mm spacing.
        lines=vtk.vtkAppendPolyData()
        for coord in np.arange(-.8,.81,.01):
            for p1,p2 in [((coord,-.8,0), (coord,.8,0)),((-.8,coord,0),(.8,coord,0))]:
                line=vtk.vtkLineSource();line.SetPoint1(*p1);line.SetPoint2(*p2);line.Update();lines.AddInputData(line.GetOutput())
        lines.Update();lm=vtk.vtkPolyDataMapper();lm.SetInputConnection(lines.GetOutputPort());la=vtk.vtkActor();la.SetMapper(lm);la.GetProperty().SetColor(.64,.70,.73);la.GetProperty().SetAmbient(1);la.GetProperty().SetDiffuse(0);ren.AddActor(la)
    camera=ren.GetActiveCamera();camera.ParallelProjectionOn();camera.SetParallelScale(halfheight);camera.SetViewUp(0,0,1);camera.SetClippingRange(.001,2.)
    light=vtk.vtkLight();light.SetLightTypeToCameraLight();light.SetIntensity(.9);ren.AddLight(light)
    fill=vtk.vtkLight();fill.SetPosition(.1,.1,.2);fill.SetFocalPoint(0,0,0);fill.SetIntensity(.5);ren.AddLight(fill)

def rotation(q):
    w,x,y,z=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],[2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
capture=vtk.vtkWindowToImageFilter();capture.SetInput(window);capture.SetInputBufferTypeToRGB();capture.ReadFrontBufferOff()
font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',20);small=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',17);title=ImageFont.truetype('C:/Windows/Fonts/msyhbd.ttc',27)
frames=[0,10,40,120,239] if args.preview else range(240);video=D/(args.terrain+'_unified_pd.mp4');writer=None if args.preview else imageio.get_writer(video,fps=20,codec='libx264',quality=8,macro_block_size=1,ffmpeg_params=['-threads','2','-movflags','+faststart','-preset','fast'])
min_margin=1e9;max_pos_error=0.;max_rot_error=0.;stats=[]
try:
    for count,frame in enumerate(frames):
        k=frame*5
        for e,ren in enumerate(renderers):
            for j,m in enumerate(matrices[e]):
                rot=rotation(poses[k,e,j,3:].astype(float))
                for a in range(3):
                    for b in range(3):m.SetElement(a,b,rot[a,b])
                    m.SetElement(a,3,float(poses[k,e,j,a]))
                m.SetElement(3,3,1.)
                max_pos_error=max(max_pos_error,max(abs(m.GetElement(a,3)-float(poses[k,e,j,a])) for a in range(3)))
                max_rot_error=max(max_rot_error,max(abs(m.GetElement(a,b)-rot[a,b]) for a in range(3) for b in range(3)))
            center=centers[k,e].astype(float);cam=ren.GetActiveCamera();cam.SetPosition(*(center+eyeoff));cam.SetFocalPoint(*center)
            rel=poses[k,e,:,:3].astype(float)-center
            min_margin=min(min_margin,float((halfheight-abs(rel@up)-radius).min()),float((halfheight*1.5-abs(rel@right)-radius).min()))
        window.Render();capture.Modified();capture.Update();pixels=vtk_to_numpy(capture.GetOutput().GetPointData().GetScalars()).reshape(1440,1440,3)[::-1]
        canvas=Image.new('RGB',(1440,1730),'#f5f7fa');draw=ImageDraw.Draw(canvas)
        draw.text((22,12),'统一 PD · 六波数 | '+('平地' if args.terrain=='flat' else '岩面统计 rough terrain'),font=title,fill='#10283f')
        draw.text((22,52),f't={times[k]:.2f} s · 1倍速 · 0.5 ms · TGS 64/16 · 全部173关节固定渐变PD',font=font,fill='#355169')
        draw.text((22,81),'新运行的 Isaac 物理轨迹 · 原 USD 模型 OpenGL 回放 · 相机跟随身体，统一比例',font=small,fill='#526a7b')
        for e in range(6):
            row,col=divmod(e,2);x=col*720;y=122+row*530;c=cells[e]
            tile=np.array(pixels[row*480:(row+1)*480,col*720:(col+1)*720],copy=True)
            assert tile.std()>5
            draw.text((x+18,y+5),f'n={c["wave"]:g}  |  Body {c["A_deg"]:.2f}° / Leg yaw {c["B_deg"]:.2f}°',font=font,fill='#10283f')
            canvas.paste(Image.fromarray(tile),(x,y+40));stats.append(dict(frame=frame,env=e,mean=float(tile.mean()),std=float(tile.std())))
        if writer:writer.append_data(np.asarray(canvas))
        if frame in (0,10,40,120,239):canvas.save(D/f'frame_{frame:03d}.png')
        receipt['frames_completed']=count+1
        if count%20==0:save();print('RENDER',frame,flush=True)
finally:
    if writer:writer.close()
assert min_margin>0 and max_pos_error<1e-12 and max_rot_error<1e-12
receipt.update(complete=True,finished_unix=time.time(),source_time_range_s=[0,11.95],fps=20,playback_speed=1.,body_binding_count=251,visual_parts=len(meshes),maximum_pose_position_error_m=max_pos_error,maximum_pose_rotation_matrix_error=max_rot_error,minimum_visual_frustum_margin_m=min_margin,camera_half_height_m=halfheight,original_geometry_source=str(meta['asset']),source_graphics_material_note='Original diffuse colors; OpenGL lighting differs from RTX. No new contact result is inferred.',manual_review_pending=True,frame_statistics=stats)
if writer:
    with imageio.get_reader(video) as rd:vidmeta=rd.get_meta_data();decoded=sum(1 for _ in rd)
    assert decoded==240 and vidmeta['fps']==20
    receipt.update(video=str(video),video_sha256=sha(video),full_decode_passed=True,decoded_frames=decoded)
save();window.Finalize();print('COMPLETE',str(D),flush=True)
